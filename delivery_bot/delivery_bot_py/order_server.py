#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from collections import deque
from delivery_bot_interfaces.msg import DeliveryTask, RobotStatus
from delivery_bot_interfaces.srv import DeliverPackage

class OrderServerNode(Node):
    def __init__(self):
        super().__init__("order_server")

        self.task_queue_ = deque()
        self.robot_status_ = "IDLE"
        self.last_status_ = ""
        self.publish_timer_ = None

        # Service Server
        self.delivery_service_ = self.create_service(
            DeliverPackage, "/deliver_package", self.deliver_package_callback)

        # Publisher
        self.task_publisher_ = self.create_publisher(DeliveryTask, "/delivery_tasks", 10)

        # Subscriber
        self.status_subscriber_ = self.create_subscription(
            RobotStatus, "/robot_status", self.status_callback, 10)

    def status_callback(self, msg: RobotStatus):
        # Erkenne Übergang von NICHT-IDLE zu IDLE
        if msg.status == "IDLE" and self.last_status_ != "IDLE":
            self.get_logger().info("Robot is IDLE. Publishing next task if available.")
            # Kurze Verzögerung für saubere Synchronisation
            if self.publish_timer_ is not None:
                self.publish_timer_.cancel()
            self.publish_timer_ = self.create_timer(0.2, self.publish_next_task_once)
        
        self.last_status_ = msg.status
        self.robot_status_ = msg.status

    def deliver_package_callback(self, request, response):
        # Akzeptiere nur neue Aufträge wenn Roboter IDLE ist
        if self.robot_status_ != "IDLE":
            self.get_logger().warn("Robot is busy. Rejecting new tasks.")
            response.success = False
            response.message = "Robot is currently busy."
            return response

        accepted_ids = []
        for task in request.tasks:
            # Validierung: Keine Duplikate und gültige Koordinaten (FR-3)
            is_duplicate = task.task_id in [t.task_id for t in self.task_queue_]
            is_valid_coords = (0 <= task.destination_x <= 11 and 0 <= task.destination_y <= 11)
            
            if not is_duplicate and is_valid_coords:
                self.task_queue_.append(task)
                accepted_ids.append(task.task_id)
            elif is_duplicate:
                self.get_logger().warn(f"Task {task.task_id} rejected: Duplicate")
            else:
                self.get_logger().warn(f"Task {task.task_id} rejected: Invalid coordinates ({task.destination_x}, {task.destination_y})")

        response.success = True
        response.message = f"Accepted {len(accepted_ids)} tasks."
        response.accepted_task_ids = accepted_ids

        self.get_logger().info(f"Accepted {len(accepted_ids)} tasks: {accepted_ids}\n")

        # Veröffentliche die erste Aufgabe nach kurzer Verzögerung
        if self.publish_timer_ is not None:
            self.publish_timer_.cancel()
        self.publish_timer_ = self.create_timer(0.5, self.publish_next_task_once)

        return response

    def publish_next_task_once(self):
        # Timer canceln (oneshot-Verhalten)
        if self.publish_timer_ is not None:
            self.publish_timer_.cancel()
            self.publish_timer_ = None
        
        self.publish_next_task()

    def publish_next_task(self):
        # Veröffentliche eine Aufgabe nach der anderen
        if self.robot_status_ == "IDLE" and len(self.task_queue_) > 0:
            task = self.task_queue_.popleft()
            self.task_publisher_.publish(task)
            self.get_logger().info(f"Publishing task: {task.task_id}\n")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OrderServerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()