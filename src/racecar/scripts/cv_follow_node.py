#!/usr/bin/env python3
"""ROS2 port of the fixed motion CV follow script."""

import math

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class CvFollowNode(Node):
    def __init__(self):
        super().__init__('cv_follow_node')
        qos_depth = 10
        #self.vel_pub = self.create_publisher(Twist, 'cv_control', qos_depth)
        self.vel_pub = self.create_publisher(Twist, '/teleop_cmd_vel', qos_depth)
        self.start_laser_pub = self.create_publisher(Bool, 'start_laser', qos_depth)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/encoder_imu_odom',
            self._odom_cb,
            qos_depth,
        )
        self.declare_parameter('drive_speed', 1530.0)
        self.declare_parameter('drive_angle', 83.0)
        self.declare_parameter('turn_speed', 1500.0)
        self.declare_parameter('turn_angle', 105.0)
        self.declare_parameter('straight_distance_m', 1.95)
        self.declare_parameter('turn_distance_m', 5.0)
        self.declare_parameter('stop_linear', 1350.0)
        self.declare_parameter('stop_angular', 90.0)
        self._stage = 0
        self._timer = self.create_timer(0.1, self._timer_cb)
        self._current_distance_sq = 0.0
        self._current_distance = 0.0
        self._has_odom = False
        self._last_position = None
        self._straight_distance = 0.0
        self._turn_distance = 0.0
        self._total_distance = 0.0
        self._finished = Future()
        self.get_logger().info('cv_follow_node initialised')

    def _odom_cb(self, msg: Odometry):
        position = msg.pose.pose.position
        self._current_distance_sq = position.x * position.x + position.y * position.y
        self._current_distance = math.sqrt(self._current_distance_sq)
        if self._last_position is not None:
            dx = position.x - self._last_position[0]
            dy = position.y - self._last_position[1]
            delta_dist = math.sqrt(dx * dx + dy * dy)
            self._total_distance += delta_dist
            if self._stage == 0:
                self._straight_distance += delta_dist
            elif self._stage == 1:
                self._turn_distance += delta_dist
        self._last_position = (position.x, position.y)
        self._has_odom = True

    def _timer_cb(self):
        now = self.get_clock().now()
        cmd = Twist()
        if self._has_odom:
            self.get_logger().info(
                'Distance (m) -> straight: %.2f, turn: %.2f, total: %.2f | origin: %.2f m'
                % (
                    self._straight_distance,
                    self._turn_distance,
                    self._total_distance,
                    self._current_distance,
                )
            )
        if self._stage == 0:
            cmd.linear.x = self.get_parameter('drive_speed').value
            cmd.angular.z = self.get_parameter('drive_angle').value
            self.vel_pub.publish(cmd)
            straight_target = self.get_parameter('straight_distance_m').value
            if self._has_odom and self._straight_distance >= straight_target:
                self._stage = 1
                self.get_logger().info(
                    'Reached straight target (segment=%.2f m, origin=%.2f m), starting turn'
                    % (self._straight_distance, self._current_distance)
                )
        elif self._stage == 1:
            cmd.linear.x = self.get_parameter('turn_speed').value
            cmd.angular.z = self.get_parameter('turn_angle').value
            self.vel_pub.publish(cmd)
            turn_target = self.get_parameter('turn_distance_m').value
            if self._has_odom and self._turn_distance >= turn_target:
                self._stage = 2
                self.get_logger().info(
                    'Reached turn target (segment=%.2f m, origin=%.2f m), stopping and starting laser'
                    % (self._turn_distance, self._current_distance)
                )
        elif self._stage == 2:
            cmd.linear.x = self.get_parameter('stop_linear').value
            cmd.angular.z = self.get_parameter('stop_angular').value
            self.vel_pub.publish(cmd)
            start_msg = Bool()
            start_msg.data = True
            self.start_laser_pub.publish(start_msg)
            self.get_logger().info('Motion sequence complete, shutting down')
            self._stage = 3
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            if not self._finished.done():
                self._finished.set_result(True)
            return
        else:
            # Already finished, keep publishing stop to be safe.
            cmd.linear.x = self.get_parameter('stop_linear').value
            cmd.angular.z = self.get_parameter('stop_angular').value
            self.vel_pub.publish(cmd)

    @property
    def finished_future(self):
        return self._finished


def main():
    rclpy.init()
    node = CvFollowNode()
    try:
        rclpy.spin_until_future_complete(node, node.finished_future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




