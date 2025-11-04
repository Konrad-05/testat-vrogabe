#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from collections import deque

# Importiere die benötigten Interfaces
from delivery_bot_interfaces.msg import DeliveryTask, RobotStatus
from delivery_bot_interfaces.srv import DeliverPackage

class OrderServerNode(Node):
    def __init__(self):
        super().__init__("order_server")

        # Aufgaben-Warteschlange [cite: 35]
        self.task_queue_ = deque()
        self.robot_status_ = "IDLE"

        # Service Server für /deliver_package [cite: 110]
        self.delivery_service_ = self.create_service(
            DeliverPackage, "/deliver_package", self.deliver_package_callback)

        # Publisher für /delivery_tasks [cite: 111]
        self.task_publisher_ = self.create_publisher(DeliveryTask, "/delivery_tasks", 10)

        # Subscriber für /robot_status, um zu wissen, wann der Roboter IDLE ist
        self.status_subscriber_ = self.create_subscription(
            RobotStatus, "/robot_status", self.status_callback, 10)

        self.get_logger().info("Order Server has been started.")

    def status_callback(self, msg: RobotStatus):
        # Prüfe, ob der Roboter gerade IDLE geworden ist
        if msg.status == "IDLE" and self.robot_status_ != "IDLE":
            self.get_logger().info("Robot is IDLE. Publishing next task if available.")
            self.publish_next_task()
        self.robot_status_ = msg.status

    def deliver_package_callback(self, request, response):
        # Akzeptiere nur neue Aufträge, wenn der Roboter IDLE ist [cite: 32]
        if self.robot_status_ != "IDLE":
            self.get_logger().warn("Robot is busy. Rejecting new tasks.")
            response.success = False
            response.message = "Robot is currently busy."
            return response

        accepted_ids = []
        for task in request.tasks: # [cite: 96]
            # TODO: Implementiere Validierung [cite: 30]
            self.task_queue_.append(task)
            accepted_ids.append(task.task_id)

        response.success = True
        response.message = f"Accepted {len(accepted_ids)} tasks."
        response.accepted_task_ids = accepted_ids # [cite: 100]

        self.get_logger().info(response.message)

        # Veröffentliche die erste Aufgabe
        self.publish_next_task()

        return response

    def publish_next_task(self):
        # Veröffentliche eine Aufgabe nach der anderen [cite: 33, 35]
        if self.robot_status_ == "IDLE" and len(self.task_queue_) > 0:
            task = self.task_queue_.popleft()
            self.task_publisher_.publish(task)
            self.get_logger().info(f"Publishing task: {task.task_id}")

def main(args=None):
    rclpy.init(args=args)
    node = OrderServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()