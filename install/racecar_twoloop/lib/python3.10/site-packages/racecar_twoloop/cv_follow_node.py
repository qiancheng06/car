#!/usr/bin/env python3
"""ROS 2 motion sequence publisher that drives the CV legs."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future


class CvFollowNode(Node):
    def __init__(self) -> None:
        super().__init__('cv_follow_node')
        qos_depth = 10
        self.output_topic = self.declare_parameter('output_topic', '/teleop_cmd_vel').value
        #self.output_topic = self.declare_parameter('output_topic', '/cv_cmd_vel').value
        self.vel_pub = self.create_publisher(Twist, str(self.output_topic), qos_depth)
        self.declare_parameter('drive_duration', 2.0)
        self.declare_parameter('drive_speed', 1525.0)
        self.declare_parameter('drive_angle', 86.5)
        self.declare_parameter('turn_duration', 0.0)
        self.declare_parameter('turn_speed', 1500.0)
        self.declare_parameter('turn_angle', 115.0)
        self.declare_parameter('stop_linear', 1350.0)
        self.declare_parameter('stop_angular', 90.0)
        self.declare_parameter('reset_duration', 1.0)
        self._stage = 0
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(0.1, self._timer_cb)
        self._finished = Future()
        self.get_logger().info('cv_follow_node initialised')

    def _timer_cb(self) -> None:
        now = self.get_clock().now()
        cmd = Twist()
        if self._stage == 0:
            cmd.linear.x = self.get_parameter('drive_speed').value
            cmd.angular.z = self.get_parameter('drive_angle').value
            self.vel_pub.publish(cmd)
            drive_duration = self.get_parameter('drive_duration').value
            target_time = self._start_time + Duration(seconds=drive_duration)
            if now >= target_time:
                self._stage = 1
                self._start_time = target_time
        elif self._stage == 1:
            cmd.linear.x = self.get_parameter('turn_speed').value
            cmd.angular.z = self.get_parameter('turn_angle').value
            self.vel_pub.publish(cmd)
            turn_duration = self.get_parameter('turn_duration').value
            target_time = self._start_time + Duration(seconds=turn_duration)
            if now >= target_time:
                self._stage = 2
                self._start_time = target_time
        elif self._stage == 2:
            cmd.linear.x = self.get_parameter('stop_linear').value
            cmd.angular.z = self.get_parameter('stop_angular').value
            self.vel_pub.publish(cmd)
            self.get_logger().info('Motion sequence complete, holding for reset')
            self._stage = 3
            self._start_time = now
        elif self._stage == 3:
            cmd.linear.x = self.get_parameter('stop_linear').value
            cmd.angular.z = self.get_parameter('stop_angular').value
            self.vel_pub.publish(cmd)
            reset_duration = self.get_parameter('reset_duration').value
            target_time = self._start_time + Duration(seconds=reset_duration)
            if now >= target_time:
                self.get_logger().info('Reset window finished, shutting down')
                self._stage = 4
                if self._timer is not None:
                    self._timer.cancel()
                    self._timer = None
                if not self._finished.done():
                    self._finished.set_result(True)
                return

    @property
    def finished_future(self) -> Future:
        return self._finished


def main() -> None:
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
