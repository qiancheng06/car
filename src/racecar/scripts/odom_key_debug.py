import msvcrt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
import select
import termios
import tty

class OdomKeyDebug(Node):
    def __init__(self):
        super().__init__("odom_key_debug")
        self.last_x = 0.0
        self.last_y = 0.0
        self.create_subscription(Odometry, "/encoder_imu_odom", self.odom_cb, 10)
        self.get_logger().info("按 P 打印 base_link_x/base_link_y/pose_norm")
    
    def odom_cb(self, msg: Odometry):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y

    def spin_with_keyboard(self):
        stdin_fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._poll_key() == "p":
                    pose_norm = math.hypot(self.last_x, self.last_y)
                    self.get_logger().info(
                        f"x={self.last_x:.3f} y={self.last_y:.3f} norm={pose_norm:.3f}")
        finally:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)

    def _poll_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1).lower()
        return None

def main():
    rclpy.init()
    node = OdomKeyDebug()
    node.spin_with_keyboard()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()