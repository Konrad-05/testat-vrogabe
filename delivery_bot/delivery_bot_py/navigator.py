#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from functools import partial

# Importiere die benötigten Interfaces
from delivery_bot_interfaces.msg import DeliveryTask, RobotStatus
from delivery_bot_interfaces.srv import ChargeBattery
from rclpy.parameter import Parameter

# NEUE Imports für Turtlesim
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator")

        # --- Parameter ---
        self.declare_parameter("battery_capacity", 100.0)
        self.declare_parameter("delivery_speed", 1.0)
        self.declare_parameter("battery_consumption_rate", 10.0)

        self.battery_capacity_ = self.get_parameter(
            "battery_capacity").get_parameter_value().double_value
        self.battery_level_ = self.battery_capacity_
        self.speed_ = self.get_parameter(
            "delivery_speed").get_parameter_value().double_value
        self.consumption_rate_ = self.get_parameter(
            "battery_consumption_rate").get_parameter_value().double_value

        # --- Interner Status ---
        self.current_pos_x_ = 0.0
        self.current_pos_y_ = 0.0
        self.current_pose_ = None
        self.current_task_ = None
        self.pending_task_ = None  # NEU: Speichert Aufgabe während des Ladens
        self.status_ = "IDLE"
        self.is_charging_ = False  # NEU: Flag für Ladezustand

        # --- Subscriber für /delivery_tasks ---
        self.task_subscriber_ = self.create_subscription(
            DeliveryTask, "/delivery_tasks", self.task_callback, 10)

        # --- Publisher für /robot_status ---
        self.status_publisher_ = self.create_publisher(RobotStatus, "/robot_status", 10)

        # --- Client für /charge_battery ---
        self.charge_client_ = self.create_client(ChargeBattery, "/charge_battery")

        # --- Publisher für Turtlesim-Steuerung ---
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # --- Subscriber für Turtlesim-Position ---
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)

        # --- Timer ---
        self.simulation_timer_ = self.create_timer(0.1, self.update_simulation) 
        self.status_timer_ = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Navigator Node (mit Turtlesim-Steuerung) has been started.")

    def pose_callback(self, msg: Pose):
        """Aktualisiert die interne Position mit den Daten von Turtlesim."""
        self.current_pose_ = msg
        self.current_pos_x_ = msg.x
        self.current_pos_y_ = msg.y

    def task_callback(self, msg: DeliveryTask):
        # Ignoriere neue Aufgaben, wenn wir laden oder bereits eine Aufgabe haben
        if self.is_charging_:
            self.get_logger().warn(f"Currently charging. Task {msg.task_id} will be queued.")
            return
        
        if self.status_ != "IDLE":
            self.get_logger().warn(f"Robot busy. Task {msg.task_id} rejected.")
            return

        # Warte, bis wir eine Position von Turtlesim haben
        if self.current_pose_ is None:
            self.get_logger().warn("No position from Turtlesim yet. Retrying in 1 second...")
            self.create_timer(1.0, lambda: self.task_callback(msg), oneshot=True)
            return

        self.get_logger().info(f"New task received: {msg.task_id} -> ({msg.destination_x:.2f}, {msg.destination_y:.2f})")

        # Berechne benötigte Energie
        distance = self.calculate_distance(msg.destination_x, msg.destination_y)
        energy_required = distance * self.consumption_rate_

        self.get_logger().info(f"Distance: {distance:.2f}, Energy required: {energy_required:.1f}%, Current battery: {self.battery_level_:.1f}%")

        # Prüfe, ob genug Batterie vorhanden ist
        if self.battery_level_ >= energy_required:
            # Genug Batterie - starte Aufgabe
            self.current_task_ = msg
            self.status_ = "MOVING_TO_DELIVERY"
            self.get_logger().info(f"Starting task {msg.task_id}")
        else:
            # Nicht genug Batterie - lade zuerst
            self.get_logger().warn(f"Insufficient energy ({self.battery_level_:.1f}% < {energy_required:.1f}%). Requesting charge.")
            self.pending_task_ = msg  # Speichere Aufgabe für später
            self.status_ = "CHARGING"
            self.is_charging_ = True
            self.call_charge_service()

    def update_simulation(self):
        """Wird alle 0.1s aufgerufen, um die Schildkröte zu steuern."""
        
        # Stoppe, wenn wir laden oder keine Aufgabe haben
        if self.is_charging_ or self.status_ != "MOVING_TO_DELIVERY" or not self.current_task_ or not self.current_pose_:
            cmd_msg = Twist()
            self.cmd_vel_publisher_.publish(cmd_msg)
            return

        # Ziel-Koordinaten
        target_x = self.current_task_.destination_x
        target_y = self.current_task_.destination_y

        # Berechne Distanz und Winkel zum Ziel
        dx = target_x - self.current_pos_x_
        dy = target_y - self.current_pos_y_
        distance_to_target = math.sqrt(dx*dx + dy*dy)

        cmd_msg = Twist()

        # Ziel erreicht
        if distance_to_target < 0.2: 
            self.cmd_vel_publisher_.publish(cmd_msg)
            self.get_logger().info(f"Task {self.current_task_.task_id} completed at ({self.current_pos_x_:.2f}, {self.current_pos_y_:.2f})")
            self.current_task_ = None
            self.status_ = "IDLE"
            return

        # P-Regler für die Steuerung
        goal_angle = math.atan2(dy, dx)
        
        # Winkelfehler berechnen
        angle_error = goal_angle - self.current_pose_.theta
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # Geschwindigkeit setzen
        if abs(angle_error) > 0.1:
            # Nur drehen
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 2.0 * angle_error
        else:
            # Fahren mit Geschwindigkeitsbegrenzung
            cmd_msg.linear.x = min(self.speed_, distance_to_target * 0.5)
            cmd_msg.angular.z = 0.5 * angle_error  # Leichte Korrektur während der Fahrt

        # Befehl senden
        self.cmd_vel_publisher_.publish(cmd_msg)

        # Batterie verbrauchen (nur wenn wir uns bewegen)
        if cmd_msg.linear.x > 0.0:
            distance_this_tick = cmd_msg.linear.x * 0.1
            battery_consumed = distance_this_tick * self.consumption_rate_
            self.battery_level_ = max(0.0, self.battery_level_ - battery_consumed)

    def publish_status(self):
        msg = RobotStatus()
        msg.task_id = self.current_task_.task_id if self.current_task_ else (
            self.pending_task_.task_id if self.pending_task_ else "")
        msg.current_x = self.current_pos_x_
        msg.current_y = self.current_pos_y_
        msg.battery_level = self.battery_level_
        msg.status = self.status_
        self.status_publisher_.publish(msg)

    def calculate_distance(self, target_x, target_y):
        """Berechnet Distanz vom aktuellen Punkt zum Ziel."""
        dx = target_x - self.current_pos_x_
        dy = target_y - self.current_pos_y_
        return math.sqrt(dx*dx + dy*dy)

    def call_charge_service(self):
        if not self.charge_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Charge service not available!")
            self.status_ = "IDLE"
            self.is_charging_ = False
            self.pending_task_ = None
            return

        self.get_logger().info("Requesting battery charge...")
        request = ChargeBattery.Request()
        future = self.charge_client_.call_async(request)
        future.add_done_callback(partial(self.charge_service_callback))

    def charge_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Battery recharged to 100%")
                self.battery_level_ = self.battery_capacity_
                self.is_charging_ = False
                self.status_ = "IDLE"
                
                # Verarbeite die gespeicherte Aufgabe
                if self.pending_task_:
                    self.get_logger().info(f"Resuming pending task: {self.pending_task_.task_id}")
                    task = self.pending_task_
                    self.pending_task_ = None
                    # Verarbeite die Aufgabe erneut (jetzt mit voller Batterie)
                    self.task_callback(task)
            else:
                self.get_logger().error("Charging failed!")
                self.is_charging_ = False
                self.status_ = "IDLE"
                self.pending_task_ = None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.is_charging_ = False
            self.status_ = "IDLE"
            self.pending_task_ = None

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()