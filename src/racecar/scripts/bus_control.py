#!/usr/bin/env python3
"""ROS2 port of the racecar coordination node from ROS1 bus_control1."""

import csv
import math
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class LaunchProcess:
    """Helper to manage external processes launched from this node."""

    def __init__(self, command: List[str], logger: Node):
        self._command = command
        self._logger = logger
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def start(self):
        with self._lock:
            if self._process and self._process.poll() is None:
                return
            self._logger.get_logger().info("Starting process: %s" % " ".join(self._command))
            # Inherit current environment so ROS_DOMAIN_ID etc. are preserved.
            self._process = subprocess.Popen(self._command, env=os.environ.copy())

    def stop(self):
        with self._lock:
            if self._process and self._process.poll() is None:
                self._logger.get_logger().info("Stopping process: %s" % " ".join(self._command))
                self._process.terminate()
                try:
                    self._process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self._logger.get_logger().warning("Process did not exit, killing: %s" % " ".join(self._command))
                    self._process.kill()
            self._process = None


class BusControlNode(Node):
    """Central coordinator that stitches together CV, LiDAR, and Nav behaviors."""

    def __init__(self):
        super().__init__('bus_control')

        qos_depth = 10
        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', qos_depth)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos_depth)

        self.create_subscription(Twist, 'laser_control', self._laser_callback, qos_depth)
        self.create_subscription(Twist, 'nav_control', self._nav_callback, qos_depth)
        self.create_subscription(Twist, 'cv_control', self._cv_callback, qos_depth)
        self.create_subscription(Bool, 'red_stop', self._red_light_callback, qos_depth)
        self.create_subscription(Bool, 'start_nav', self._start_nav_callback, qos_depth)
        self.create_subscription(Bool, 'start_change', self._start_change_callback, qos_depth)
        self.create_subscription(Bool, 'start_laser', self._start_laser_callback, qos_depth)
        self.create_subscription(Odometry, '/encoder_imu_odom', self._odom_callback, qos_depth)

        self.state_lock = threading.Lock()
        self.state = 'cv'
        self.nav_active = False
        self.is_red_light_pending = False
        self.is_change_pending = False

        self.last_pose: Optional[Tuple[float, float, float, float]] = None
        self.last_record_xy: Optional[Tuple[float, float]] = None
        self.first_point_recorded = False
        self.point_counter = 0
        self.red_reference_x = 0.0
        self.nav_red_stop_done = False

        self.nav_points: List[Tuple[float, float, float, float]] = []
        self.nav_points_lock = threading.Lock()

        self.nav_point_file = Path(
            self.declare_parameter('nav_point_file', str(Path.home() / 'racecar_nav_points.csv'))
            .get_parameter_value().string_value)
        self.nav_point_file.parent.mkdir(parents=True, exist_ok=True)
        self.nav_point_file.write_text('')

        self.map_prefix = Path(
            self.declare_parameter('map_prefix', str(Path.home() / 'racecar_map')).get_parameter_value().string_value)
        self.map_prefix.parent.mkdir(parents=True, exist_ok=True)
        self.map_saved_event = threading.Event()
        self.map_save_timeout = (
            self.declare_parameter('map_save_timeout', 10.0).get_parameter_value().double_value)
        self.map_save_retries = int(
            self.declare_parameter('map_save_retries', 3).get_parameter_value().integer_value)

        self.mapping_process = LaunchProcess(
            ['ros2', 'launch', 'slam_gmapping', 'slam_gmapping.launch.py'],
            self)
        self.nav_process = LaunchProcess(
            ['ros2', 'launch', 'racecar', 'Run_nav.launch.py', f'map:={self._map_yaml_path()}'],
            self)

        self.mapping_process.start()

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_thread: Optional[threading.Thread] = None

        self.create_timer(0.1, self._record_timer_cb)
        self.shutdown_requested = False
        self.create_timer(0.5, self._shutdown_timer_cb)

        self.get_logger().info('bus_control node initialised')

    # ------------------------------------------------------------------
    # Publisher helpers
    # ------------------------------------------------------------------

    def _publish_cmd(self, msg: Twist):
        self.cmd_pub.publish(msg)

    def _publish_stop(self, linear: float, angular: float):
        stop_msg = Twist()
        stop_msg.linear.x = linear
        stop_msg.angular.z = angular
        self._publish_cmd(stop_msg)

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _laser_callback(self, msg: Twist):
        with self.state_lock:
            current_state = self.state
        if current_state in ('laser', 'laser1'):
            self._publish_cmd(msg)
        elif current_state == 'stop1':
            self._publish_stop(1500.0, 90.0)
        elif current_state == 'stop2':
            self._publish_stop(1500.0, 90.0)

    def _nav_callback(self, msg: Twist):
        with self.state_lock:
            current_state = self.state
        if current_state == 'nav':
            self._publish_cmd(msg)
        elif current_state in ('stop3', 'stop4'):
            self._publish_stop(1500.0, 90.0)

    def _cv_callback(self, msg: Twist):
        with self.state_lock:
            if self.state != 'cv':
                return
        self._publish_cmd(msg)

    def _red_light_callback(self, msg: Bool):
        if not msg.data:
            return
        with self.state_lock:
            if not self.nav_active:
                self.state = 'stop1'
                self.is_red_light_pending = True
                threading.Thread(target=self._handle_first_red_light, daemon=True).start()
            else:
                if not self.nav_red_stop_done:
                    self.state = 'stop3'
                    self.nav_red_stop_done = True
                    threading.Thread(target=self._resume_from_nav_red_light, daemon=True).start()

    def _start_nav_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('start_nav signal received (unused placeholder)')

    def _start_change_callback(self, msg: Bool):
        if not msg.data:
            return
        with self.state_lock:
            if self.is_change_pending:
                return
            self.state = 'stop2'
            self.is_change_pending = True
        threading.Thread(target=self._handle_changeover, daemon=True).start()

    def _start_laser_callback(self, msg: Bool):
        if not msg.data:
            return
        with self.state_lock:
            if self.state == 'cv':
                self.state = 'laser'
                self.get_logger().info('Laser control phase started')

    def _odom_callback(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        self.last_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            orientation.z,
            orientation.w,
        )

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------

    def _record_timer_cb(self):
        if self.last_pose is None:
            return
        x, y, z, w = self.last_pose
        with self.state_lock:
            current_state = self.state

        if current_state == 'cv':
            self._handle_cv_record(x, y, z, w)
        elif current_state in ('laser', 'laser1', 'stop1'):
            self._handle_laser_record(x, y, z, w)
        elif current_state == 'stop2':
            # waiting for changeover thread to finish
            pass
        elif current_state == 'nav':
            self._handle_nav_stage(x, y)

    def _shutdown_timer_cb(self):
        if self.shutdown_requested:
            self.get_logger().info('Tasks complete, shutting down...')
            self.destroy_node()
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Recording helpers
    # ------------------------------------------------------------------

    def _handle_cv_record(self, x: float, y: float, z: float, w: float):
        if not self.first_point_recorded:
            self._record_point(x, y, z, w)
            self.first_point_recorded = True
            self.last_record_xy = (x, y)
            return

        if self.last_record_xy is None:
            self.last_record_xy = (x, y)
            return

        if self._distance(self.last_record_xy, (x, y)) >= 0.5:
            self._record_point(x, y, z, w)
            self.last_record_xy = (x, y)

    def _handle_laser_record(self, x: float, y: float, z: float, w: float):
        if self.is_red_light_pending:
            self.is_red_light_pending = False
            adjusted_y = y + 0.2
            self.red_reference_x = x
            self._record_point(x, adjusted_y, z, w)
            threading.Thread(target=self._save_map_once, daemon=True).start()
            return

        if self.last_record_xy is None:
            self.last_record_xy = (x, y)
            return

        if self._distance(self.last_record_xy, (x, y)) >= 1.0:
            y_adjusted = y
            if self.point_counter in (15, 16, 17):
                y_adjusted += 0.25
            self._record_point(x, y_adjusted, z, w)
            self.last_record_xy = (x, y)
            self.point_counter += 1
            self.get_logger().info(f'Recorded waypoint {self.point_counter} at ({x:.2f}, {y_adjusted:.2f})')

    def _handle_nav_stage(self, x: float, y: float):
        if self.nav_red_stop_done:
            return
        if (self.red_reference_x - 0.65) <= x <= (self.red_reference_x + 0.2) and -1.0 <= y <= 1.0:
            with self.state_lock:
                self.state = 'stop3'
            threading.Thread(target=self._resume_from_nav_red_light, daemon=True).start()

    def _record_point(self, x: float, y: float, z: float, w: float):
        with self.nav_points_lock:
            self.nav_points.append((x, y, z, w))
            with self.nav_point_file.open('a', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([x, y, z, w])

    # ------------------------------------------------------------------
    # Long-running handlers
    # ------------------------------------------------------------------

    def _handle_first_red_light(self):
        self.get_logger().info('Red light stop (lap 1) engaged')
        self._publish_stop(1500.0, 90.0)
        time.sleep(3.0)
        with self.state_lock:
            if self.state == 'stop1':
                self.state = 'laser1'
        self.get_logger().info('Resuming after red light')

    def _resume_from_nav_red_light(self):
        self._publish_stop(1500.0, 90.0)
        time.sleep(3.0)
        with self.state_lock:
            if self.state == 'stop3':
                self.state = 'nav'
        self.get_logger().info('Resuming navigation after red light hold')

    def _handle_changeover(self):
        self.get_logger().info('Changeover sequence triggered')
        self._publish_stop(1500.0, 90.0)
        z = 0.0
        w = 1.0
        if self.last_pose:
            _, _, z, w = self.last_pose
        manual_points = [(-7.5, 0.25), (-4.2, 0.35), (-3.1, 0.72)]
        for px, py in manual_points:
            self._record_point(px, py, z, w)
        self.last_record_xy = manual_points[-1]
        self.mapping_process.stop()
        time.sleep(0.5)
        self._publish_initial_pose()
        self.nav_active = True
        self._start_navigation_thread()
        with self.state_lock:
            self.state = 'nav'
        self.get_logger().info('Navigation phase begins')

    def _publish_initial_pose(self):
        if not self.last_pose:
            self.get_logger().warning('Cannot publish initial pose, odom not ready')
            return
        x, y, z, w = self.last_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Initial pose published at (%.2f, %.2f)' % (x, y))

    def _save_map_once(self):
        if self.map_saved_event.is_set():
            return
        map_base = str(self.map_prefix)
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_base,
            '--ros-args',
            '-p', 'map_subscribe_transient_local:=false',
            '-p', f'save_map_timeout:={self.map_save_timeout}',
        ]

        for attempt in range(1, max(1, self.map_save_retries) + 1):
            self.get_logger().info(
                f'Saving map attempt {attempt}/{self.map_save_retries} via: {" ".join(command)}'
            )
            try:
                subprocess.run(command, check=True, env=os.environ.copy())
                self.map_saved_event.set()
                self.get_logger().info('Map saved to %s' % self._map_yaml_path())
                return
            except subprocess.CalledProcessError as exc:
                self.get_logger().warning(f'Map save attempt {attempt} failed: {exc}')
                time.sleep(1.0)
            except FileNotFoundError as exc:
                self.get_logger().error(f'Map saver command missing: {exc}')
                break

        if not self.map_saved_event.is_set():
            self.get_logger().error(
                f'Map could not be saved after {self.map_save_retries} attempts'
            )

    def _start_navigation_thread(self):
        if self.nav_thread and self.nav_thread.is_alive():
            return
        self.nav_thread = threading.Thread(target=self._navigation_worker, daemon=True)
        self.nav_thread.start()

    def _navigation_worker(self):
        if not self.map_saved_event.wait(timeout=60.0):
            self.get_logger().error('Map was not saved in time, cannot start navigation')
            return
        self.nav_process.start()
        time.sleep(5.0)
        self.get_logger().info('Waiting for navigate_to_pose action server')
        if not self.nav_action_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('navigate_to_pose action server unavailable')
            return

        with self.nav_points_lock:
            goals = list(self.nav_points)

        for idx, (x, y, z, w) in enumerate(goals):
            goal = NavigateToPose.Goal()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = z
            goal.pose.pose.orientation.w = w

            send_future = self.nav_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warning(f'Goal {idx} not accepted')
                continue
            self.get_logger().info(f'Goal {idx} accepted')

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            if result and result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Goal {idx} reached')
            else:
                status = result.status if result else 'unknown'
                self.get_logger().warning(f'Goal {idx} finished with status {status}')

        with self.state_lock:
            self.state = 'stop4'
        self._publish_stop(1500.0, 90.0)
        self.shutdown_requested = True

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _map_yaml_path(self) -> str:
        return str(self.map_prefix.with_suffix('.yaml'))

    # ------------------------------------------------------------------
    # Lifecycle overrides
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.mapping_process.stop()
        self.nav_process.stop()
        return super().destroy_node()


def main():
    rclpy.init()
    node = BusControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
