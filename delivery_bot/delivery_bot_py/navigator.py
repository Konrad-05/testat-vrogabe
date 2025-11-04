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
        self.declare_parameter("delivery_speed", 1.0) # Einheiten pro Sekunde
        self.declare_parameter("battery_consumption_rate", 10.0) # % pro Einheit

        self.battery_level_ = self.get_parameter(
            "battery_capacity").get_parameter_value().double_value
        self.speed_ = self.get_parameter(
            "delivery_speed").get_parameter_value().double_value
        self.consumption_rate_ = self.get_parameter(
            "battery_consumption_rate").get_parameter_value().double_value

        # --- Interner Status ---
        self.current_pos_x_ = 0.0 # Wird jetzt von /turtle1/pose aktualisiert
        self.current_pos_y_ = 0.0 # Wird jetzt von /turtle1/pose aktualisiert
        self.current_pose_ = None  # Speichert die gesamte Pose (inkl. Theta)
        self.current_task_ = None
        self.status_ = "IDLE" 

        # --- Subscriber für /delivery_tasks ---
        self.task_subscriber_ = self.create_subscription(
            DeliveryTask, "/delivery_tasks", self.task_callback, 10)

        # --- Publisher für /robot_status ---
        self.status_publisher_ = self.create_publisher(RobotStatus, "/robot_status", 10)

        # --- Client für /charge_battery ---
        self.charge_client_ = self.create_client(ChargeBattery, "/charge_battery")

        # --- NEU: Publisher für Turtlesim-Steuerung ---
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # --- NEU: Subscriber für Turtlesim-Position ---
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)

        # --- Timer ---
        # simulation_timer_ wird zum "Regler" (Control Loop)
        self.simulation_timer_ = self.create_timer(0.1, self.update_simulation) 
        self.status_timer_ = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Navigator Node (mit Turtlesim-Steuerung) has been started.")

    # --- NEUE Callback-Funktion für /turtle1/pose ---
    def pose_callback(self, msg: Pose):
        """Aktualisiert die interne Position mit den Daten von Turtlesim."""
        self.current_pose_ = msg
        self.current_pos_x_ = msg.x
        self.current_pos_y_ = msg.y

    def task_callback(self, msg: DeliveryTask):
        if self.status_ == "IDLE":
            # Warte, bis wir eine Position von Turtlesim haben
            if self.current_pose_ is None:
                self.get_logger().warn("No position from Turtlesim yet. Waiting...")
                # Versuche es in 1 Sekunde erneut
                self.create_timer(1.0, lambda: self.task_callback(msg), oneshot=True)
                return

            self.current_task_ = msg
            self.get_logger().info(f"New task received: {msg.task_id}")

            distance = self.calculate_distance(msg.destination_x, msg.destination_y)
            energy_required = distance * self.consumption_rate_

            if self.battery_level_ >= energy_required:
                self.status_ = "MOVING_TO_DELIVERY"
            else:
                self.get_logger().warn("Insufficient energy. Requesting charge.")
                self.status_ = "CHARGING"
                self.call_charge_service()

    # --- ANGEPASSTE update_simulation (Jetzt der Regler) ---
    def update_simulation(self):
        """Wird alle 0.1s aufgerufen, um die Schildkröte zu steuern."""
        
        # Stoppe, wenn wir keine Aufgabe oder keine Pose haben
        if self.status_ != "MOVING_TO_DELIVERY" or not self.current_task_ or not self.current_pose_:
            # Sende einen Stopp-Befehl
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

        # --- Ziel-Erreichungs-Logik ---
        # Ziel fast erreicht
        if distance_to_target < 0.2: 
            self.cmd_vel_publisher_.publish(cmd_msg) # Stopp
            self.get_logger().info(f"Task {self.current_task_.task_id} completed.")
            self.current_task_ = None
            self.status_ = "IDLE"
            return

        # --- P-Regler für die Steuerung ---
        
        # 1. Winkel zum Ziel berechnen
        goal_angle = math.atan2(dy, dx)
        
        # 2. Winkelfehler berechnen (Differenz zum aktuellen Winkel der Schildkröte)
        # Normalisieren des Fehlers auf -pi bis +pi
        angle_error = goal_angle - self.current_pose_.theta
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # 3. Geschwindigkeit setzen
        
        # Nur drehen, wenn der Winkel zu groß ist
        if abs(angle_error) > 0.1:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 2.0 * angle_error # Winkelgeschwindigkeit (kP = 2.0)
        else:
            # Fahren, wenn wir richtig ausgerichtet sind
            # Begrenze die Geschwindigkeit auf die max. Geschwindigkeit
            cmd_msg.linear.x = min(self.speed_, distance_to_target * 0.5) # (kP = 0.5)
            cmd_msg.angular.z = 0.0

        # 4. Befehl senden
        self.cmd_vel_publisher_.publish(cmd_msg)

        # 5. Batterie verbrauchen (basierend auf der tatsächlichen linearen Geschwindigkeit)
        if cmd_msg.linear.x > 0.0:
            distance_this_tick = cmd_msg.linear.x * 0.1 # 0.1s ist die Timer-Periode
            self.battery_level_ -= distance_this_tick * self.consumption_rate_


    def publish_status(self):
        msg = RobotStatus()
        msg.task_id = self.current_task_.task_id if self.current_task_ else ""
        msg.current_x = self.current_pos_x_
        msg.current_y = self.current_pos_y_
        msg.battery_level = self.battery_level_
        msg.status = self.status_
        self.status_publisher_.publish(msg)

    # --- NEU: calculate_distance nutzt jetzt die interne (reale) Position ---
    def calculate_distance(self, target_x, target_y):
        """Berechnet Distanz vom aktuellen (realen) Punkt zum Ziel."""
        dx = target_x - self.current_pos_x_
        dy = target_y - self.current_pos_y_
        return math.sqrt(dx*dx + dy*dy)

    def call_charge_service(self):
        if not self.charge_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Charge service not available.")
            self.status_ = "IDLE" 
            return

        request = ChargeBattery.Request()
        future = self.charge_client_.call_async(request)
        future.add_done_callback(partial(self.charge_service_callback))

    def charge_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Recharging complete.")
                self.battery_level_ = 100.0
                self.status_ = "IDLE"
                
                if self.current_task_:
                    self.get_logger().info("Resuming pending task.")
                    # Triggere die Aufgabenprüfung erneut
                    self.task_callback(self.current_task_) 
            else:
                self.get_logger().error("Charging service failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()