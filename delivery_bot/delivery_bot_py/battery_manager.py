#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from delivery_bot_interfaces.msg import RobotStatus, BatteryAlert
from delivery_bot_interfaces.srv import ChargeBattery

class BatteryManagerNode(Node):
    def __init__(self):
        super().__init__("battery_manager")

        self.declare_parameter("low_battery_threshold", 20.0)
        self.threshold_ = self.get_parameter("low_battery_threshold").value

        self.create_subscription(RobotStatus, "/robot_status", self.status_callback, 10)
        self.alert_pub_ = self.create_publisher(BatteryAlert, "/battery_alert", 10)
        self.create_service(ChargeBattery, "/charge_battery", self.charge_callback)
        
        self.last_warned_ = False

    def status_callback(self, msg: RobotStatus):
        # Nur eine Warnung ausgeben wenn Batterie niedrig wird
        if msg.battery_level < self.threshold_ and not self.last_warned_:
            alert = BatteryAlert()
            alert.low_battery = True
            alert.current_level = msg.battery_level
            self.alert_pub_.publish(alert)
            self.last_warned_ = True
        elif msg.battery_level >= self.threshold_:
            self.last_warned_ = False

    def charge_callback(self, request, response):
        self.get_logger().info("Starting battery charge...")
        # Instantanes Laden (FR-20)
        self.get_logger().info("Battery charging completed\n")
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatteryManagerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()