#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Importiere die benutzerdefinierte Nachricht
from delivery_bot_interfaces.msg import RobotStatus 

class StatusDisplayNode(Node):
    def __init__(self):
        super().__init__("status_display")
        # Erstelle Subscriber für /robot_status [cite: 125]
        self.status_subscriber_ = self.create_subscription(
            RobotStatus, "/robot_status", self.status_callback, 10)
        self.get_logger().info("Status Display Node has been started.")

    def status_callback(self, msg: RobotStatus):
        # Zeige die empfangenen Statusinformationen an 
        self.get_logger().info(
            f"--- ROBOT STATUS ---\n"
            f"Task ID: {msg.task_id}\n"
            f"Position: ({msg.current_x:.2f}, {msg.current_y:.2f})\n"
            f"Battery: {msg.battery_level:.1f}%\n"
            f"Status: {msg.status}\n"
            f"----------------------"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StatusDisplayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()