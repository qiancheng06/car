#!/usr/bin/env python3
"""CV-based fixed motion segment with reusable start trigger."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class CvFollowNode(Node):
    def __init__(self) -> None:
        super().__init__('cv_follow_node')
        qos = 10

        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', qos)
        self.start_laser_pub = self.create_publisher(Bool, 'start_laser', qos)
        self.create_subscription(Odometry, '/encoder_imu_odom', self._odom_cb, qos)
        self.create_subscription(Bool, 'start_cv', self._start_cv_cb, qos)

        self.declare_parameter('drive_speed', 1530.0)
        self.declare_parameter('drive_angle', 83.0)
        self.declare_parameter('turn_speed', 1500.0)
        self.declare_parameter('turn_angle', 105.0)
        self.declare_parameter('straight_distance_m', 1.95)
        self.declare_parameter('turn_distance_m', 5.0)
        self.declare_parameter('stop_linear', 1350.0)
        self.declare_parameter('stop_angular', 90.0)
        self.declare_parameter('auto_start', True)

        self._stage = -1
        self._active_run = False
        self._current_origin = 0.0
        self._has_odom = False

        self.create_timer(0.1, self._timer_cb)

        if self.get_parameter('auto_start').value:
            self._begin_run()

        self.get_logger().info('cv_follow_node ready')

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _start_cv_cb(self, msg: Bool) -> None:
        if msg.data:
            self._begin_run()

    def _odom_cb(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self._current_origin = math.hypot(position.x, position.y)
        self._has_odom = True

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------

    def _begin_run(self) -> None:
        if self._active_run:
            return
        self._active_run = True
        self._stage = 0
        self.get_logger().info('CV segment started')

    def _timer_cb(self) -> None:
        if not self._active_run:
            return
        if not self._has_odom:
            return

        cmd = Twist()
        if self._stage == 0:
            cmd.linear.x = self.get_parameter('drive_speed').value
            cmd.angular.z = self.get_parameter('drive_angle').value
            self.cmd_pub.publish(cmd)
            target = self.get_parameter('straight_distance_m').value
            if self._current_origin >= target:
                self._stage = 1
                self.get_logger().info(
                    'Straight segment done (origin %.2f m)' % self._current_origin
                )
        elif self._stage == 1:
            cmd.linear.x = self.get_parameter('turn_speed').value
            cmd.angular.z = self.get_parameter('turn_angle').value
            self.cmd_pub.publish(cmd)
            target = self.get_parameter('turn_distance_m').value
            if self._current_origin >= target:
                self._stage = 2
                self.get_logger().info(
                    'Turn segment done (origin %.2f m)' % self._current_origin
                )
        elif self._stage == 2:
            cmd.linear.x = self.get_parameter('stop_linear').value
            cmd.angular.z = self.get_parameter('stop_angular').value
            self.cmd_pub.publish(cmd)
            start_msg = Bool()
            start_msg.data = True
            self.start_laser_pub.publish(start_msg)
            self.get_logger().info('CV segment finished, waiting for next trigger')
            self._reset_state()

    def _reset_state(self) -> None:
        self._stage = -1
        self._active_run = False
        self._current_origin = 0.0


def main() -> None:
    rclpy.init()
    node = CvFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
