#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from functools import partial
from delivery_bot_interfaces.msg import DeliveryTask, RobotStatus, BatteryAlert
from delivery_bot_interfaces.srv import ChargeBattery
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator")

        # Parameter
        self.declare_parameter("battery_capacity", 100.0)
        self.declare_parameter("delivery_speed", 1.0)
        self.declare_parameter("battery_consumption_rate", 10.0)

        self.battery_level_ = self.get_parameter("battery_capacity").value
        self.speed_ = self.get_parameter("delivery_speed").value
        self.consumption_rate_ = self.get_parameter("battery_consumption_rate").value

        # Status
        self.current_pose_ = None
        self.current_task_ = None
        self.status_ = "IDLE"
        self.task_energy_needed_ = 0.0

        # ROS Interfaces
        self.task_subscriber_ = self.create_subscription(
            DeliveryTask, "/delivery_tasks", self.task_callback, 10)
        self.status_publisher_ = self.create_publisher(RobotStatus, "/robot_status", 10)
        self.charge_client_ = self.create_client(ChargeBattery, "/charge_battery")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        # NEU: Subscriber für Battery Alert
        from delivery_bot_interfaces.msg import BatteryAlert
        self.alert_subscriber_ = self.create_subscription(
            BatteryAlert, "/battery_alert", self.battery_alert_callback, 10)

        # Timer nur für Control Loop
        self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg: Pose):
        self.current_pose_ = msg

    def battery_alert_callback(self, msg):
        """Reagiere auf Low-Battery Warnung"""
        if msg.low_battery:
            self.get_logger().warn(f"LOW BATTERY ALERT: {msg.current_level:.1f}%")

    def task_callback(self, msg: DeliveryTask):
        if self.status_ != "IDLE" or not self.current_pose_:
            return

        # OUTPUT 1: Aktuelle Position -> Ziel + Akkustand
        self.get_logger().info(
            f"NEW TASK: {msg.task_id}\n"
            f"Current Position: ({self.current_pose_.x:.2f}, {self.current_pose_.y:.2f})\n"
            f"Target Position:  ({msg.destination_x:.2f}, {msg.destination_y:.2f})\n"
            f"Battery Level: {self.battery_level_:.1f}%\n"
        )
        
        # Publiziere Status
        self.publish_status()

        # Berechne Energie
        distance = self.calculate_distance(msg.destination_x, msg.destination_y)
        energy_needed = distance * self.consumption_rate_

        # OUTPUT 2: Batterie-Check
        if self.battery_level_ >= energy_needed:
            self.get_logger().info(
                f"Battery sufficient ({self.battery_level_:.1f}% >= {energy_needed:.1f}%)\n"
                f"Starting delivery...\n"
            )
            self.start_task(msg, energy_needed)
        else:
            self.get_logger().info(
                f"Battery insufficient ({self.battery_level_:.1f}% < {energy_needed:.1f}%)\n"
                f"Charging battery...\n"
            )
            self.status_ = "CHARGING"
            self.publish_status()
            self.charge_and_start(msg, energy_needed)

    def start_task(self, task, energy_needed):
        self.current_task_ = task
        self.task_energy_needed_ = energy_needed
        self.status_ = "MOVING_TO_DELIVERY"
        self.publish_status()

    def charge_and_start(self, task, energy_needed):
        if not self.charge_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Charge service unavailable!")
            self.status_ = "IDLE"
            return

        future = self.charge_client_.call_async(ChargeBattery.Request())
        future.add_done_callback(lambda f: self.on_charged(f, task, energy_needed))

    def on_charged(self, future, task, energy_needed):
        try:
            if future.result().success:
                self.battery_level_ = 100.0
                self.get_logger().info(
                    f"Battery charged to 100%\n"
                    f"Starting delivery...\n"
                )
                self.start_task(task, energy_needed)
        except Exception as e:
            self.get_logger().error(f"Charging failed: {e}")
            self.status_ = "IDLE"

    def control_loop(self):
        if self.status_ != "MOVING_TO_DELIVERY" or not self.current_task_ or not self.current_pose_:
            self.cmd_vel_publisher_.publish(Twist())
            return

        dx = self.current_task_.destination_x - self.current_pose_.x
        dy = self.current_task_.destination_y - self.current_pose_.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Ziel erreicht
        if distance < 0.2:
            self.cmd_vel_publisher_.publish(Twist())
            # Ziehe Batterie ab
            self.battery_level_ -= self.task_energy_needed_
            
            # OUTPUT 3: Ankunft + Akkustand nach Verbrauch
            self.get_logger().info(
                f"TASK COMPLETED: {self.current_task_.task_id}\n"
                f"Arrived at: ({self.current_pose_.x:.2f}, {self.current_pose_.y:.2f})\n"
                f"Battery after delivery: {self.battery_level_:.1f}%\n"
            )
            
            self.current_task_ = None
            self.status_ = "IDLE"
            self.publish_status()
            return

        # Fahre zum Ziel
        goal_angle = math.atan2(dy, dx)
        angle_error = goal_angle - self.current_pose_.theta
        while angle_error > math.pi: angle_error -= 2*math.pi
        while angle_error < -math.pi: angle_error += 2*math.pi

        cmd = Twist()
        if abs(angle_error) > 0.1:
            cmd.angular.z = 2.0 * angle_error
        else:
            cmd.linear.x = min(self.speed_, distance * 0.5)
            cmd.angular.z = 0.5 * angle_error
        
        self.cmd_vel_publisher_.publish(cmd)

    def publish_status(self):
        msg = RobotStatus()
        msg.task_id = self.current_task_.task_id if self.current_task_ else ""
        msg.current_x = self.current_pose_.x if self.current_pose_ else 0.0
        msg.current_y = self.current_pose_.y if self.current_pose_ else 0.0
        msg.battery_level = self.battery_level_
        msg.status = self.status_
        self.status_publisher_.publish(msg)

    def calculate_distance(self, target_x, target_y):
        if not self.current_pose_:
            return 0.0
        dx = target_x - self.current_pose_.x
        dy = target_y - self.current_pose_.y
        return math.sqrt(dx*dx + dy*dy)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NavigatorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()