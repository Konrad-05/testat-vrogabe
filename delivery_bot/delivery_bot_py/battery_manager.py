#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Importiere die benötigten Interfaces
from delivery_bot_interfaces.msg import DeliveryTask, RobotStatus, BatteryAlert
from delivery_bot_interfaces.srv import ChargeBattery
from rclpy.parameter import Parameter

class BatteryManagerNode(Node):
    def __init__(self):
        super().__init__("battery_manager")

        # Parameter für niedrigen Batteriestand [cite: 74]
        self.declare_parameter("low_battery_threshold", 20.0)
        self.low_battery_threshold_ = self.get_parameter(
            "low_battery_threshold").get_parameter_value().double_value

        # Subscriber für /robot_status [cite: 119]
        self.status_subscriber_ = self.create_subscription(
            RobotStatus, "/robot_status", self.status_callback, 10)

        # Publisher für /battery_alert [cite: 119]
        self.alert_publisher_ = self.create_publisher(
            BatteryAlert, "/battery_alert", 10)

        # Service Server für /charge_battery [cite: 121]
        self.charge_service_ = self.create_service(
            ChargeBattery, "/charge_battery", self.charge_battery_callback)

        self.get_logger().info(f"Battery Manager started. Low battery threshold: {self.low_battery_threshold_}%")

    def status_callback(self, msg: RobotStatus):
        # Überwache Batteriestand [cite: 51]
        if msg.battery_level < self.low_battery_threshold_:
            # Sende Warnung, wenn Batterie niedrig ist [cite: 54]
            alert_msg = BatteryAlert()
            alert_msg.low_battery = True
            alert_msg.current_level = msg.battery_level
            self.alert_publisher_.publish(alert_msg)
            self.get_logger().warn(f"Low battery warning! Level: {msg.battery_level:.1f}%")

    def charge_battery_callback(self, request, response):
        # Simuliere sofortiges Aufladen [cite: 58]
        self.get_logger().info("Charging battery...")
        # (In einer echten Simulation würde dies den Navigator-Node aktualisieren)
        # Hier signalisieren wir einfach den Erfolg.
        self.get_logger().info("Battery fully charged.")
        response.success = True # [cite: 59]
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()