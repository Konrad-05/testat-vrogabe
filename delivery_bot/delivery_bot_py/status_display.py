#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from delivery_bot_interfaces.msg import RobotStatus 

class StatusDisplayNode(Node):
    def __init__(self):
        super().__init__("status_display")
        self.create_subscription(RobotStatus, "/robot_status", self.status_callback, 10)
        self.last_status_ = ""

    def status_callback(self, msg: RobotStatus):
        if msg.status != self.last_status_ and msg.status in ["MOVING_TO_DELIVERY", "CHARGING"]:
            status_text = "MOVING" if msg.status == "MOVING_TO_DELIVERY" else "CHARGING"
            self.get_logger().info(f"   Status: {status_text}")
            self.last_status_ = msg.status
        elif msg.status == "IDLE" and self.last_status_ != "IDLE":
            self.last_status_ = msg.status

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StatusDisplayNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()