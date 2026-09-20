#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from std_msgs.msg import Float64


class Fase1ActuacionPublisher(Node):
    def __init__(self):
        super().__init__("fase1_actuacion_publisher")
        self.declare_parameter("actuation_mode", "force")
        self.declare_parameter("force_value", 14.0)
        self.declare_parameter("torque_x_nm", 0.01)
        self.declare_parameter("torque_z_nm", 0.01)
        self.declare_parameter("idle_duration_sec", 10.0)
        self.declare_parameter("actuation_duration_sec", 20.0)
        self.actuation_mode = self.get_parameter("actuation_mode").value
        self.force_value = self.get_parameter("force_value").value
        self.torque_x_nm = self.get_parameter("torque_x_nm").value
        self.torque_z_nm = self.get_parameter("torque_z_nm").value
        self.idle_duration_sec = self.get_parameter("idle_duration_sec").value
        self.actuation_duration_sec = self.get_parameter(
            "actuation_duration_sec").value
        self.total_duration_sec = (
            self.idle_duration_sec + self.actuation_duration_sec)
        if self.actuation_mode not in ("force", "torque_x", "torque_z"):
            raise ValueError(f"Unsupported actuation_mode: {self.actuation_mode}")
        if self.idle_duration_sec < 0.0 or self.actuation_duration_sec <= 0.0:
            raise ValueError("Actuation durations must be non-negative and positive")
        self.force_publisher = self.create_publisher(
            Float64, "/dron_1/control/tray/fuerza", 10)
        self.torque_publisher = self.create_publisher(
            Vector3Stamped, "/dron_1/control/tray/torque", 10)
        self.started_at = time.monotonic()
        self.last_phase = None
        self.timer = self.create_timer(0.02, self.publish_command)
        if self.actuation_mode == "force":
            start_message = (
                f"[F1-ACTUATION-START] mode=force force=0N for "
                f"{self.idle_duration_sec:g}s, then "
                f"force={self.force_value:g}N for "
                f"{self.actuation_duration_sec:g}s, torque=(0,0,0), "
                f"total_duration={self.total_duration_sec:g}s")
        elif self.actuation_mode == "torque_x":
            start_message = (
                f"[F1-ACTUATION-START] mode=torque_x force=0N, torque_x=0 "
                f"for {self.idle_duration_sec:g}s, then "
                f"torque_x={self.torque_x_nm:g}N*m for "
                f"{self.actuation_duration_sec:g}s, "
                f"total_duration={self.total_duration_sec:g}s")
        else:
            start_message = (
                f"[F1-ACTUATION-START] mode=torque_z force=0N, torque_z=0 "
                f"for {self.idle_duration_sec:g}s, then "
                f"torque_z={self.torque_z_nm:g}N*m for "
                f"{self.actuation_duration_sec:g}s, "
                f"total_duration={self.total_duration_sec:g}s")
        self.get_logger().warn(start_message)

    def publish_command(self):
        elapsed = time.monotonic() - self.started_at
        force = 0.0
        torque_x = 0.0
        torque_z = 0.0
        if elapsed < self.idle_duration_sec:
            phase = "idle"
        elif elapsed < self.total_duration_sec and self.actuation_mode == "force":
            phase = f"force_{self.force_value:g}N"
            force = self.force_value
        elif elapsed < self.total_duration_sec and self.actuation_mode == "torque_x":
            phase = f"torque_x_{self.torque_x_nm:g}Nm"
            torque_x = self.torque_x_nm
        elif elapsed < self.total_duration_sec:
            phase = f"torque_z_{self.torque_z_nm:g}Nm"
            torque_z = self.torque_z_nm
        else:
            phase = "finished"

        if phase != self.last_phase:
            self.last_phase = phase
            self.get_logger().warn(
                f"[F1-ACTUATION-PHASE] phase={phase} elapsed={elapsed:.3f} "
                f"force={force:.3f} torque=({torque_x:.3f},0,{torque_z:.3f})")

        force_message = Float64()
        force_message.data = force
        self.force_publisher.publish(force_message)

        torque_message = Vector3Stamped()
        torque_message.header.stamp = self.get_clock().now().to_msg()
        torque_message.header.frame_id = "cuerpo"
        torque_message.vector.x = torque_x
        torque_message.vector.y = 0.0
        torque_message.vector.z = torque_z
        self.torque_publisher.publish(torque_message)

        if phase == "finished":
            self.get_logger().warn(
                f"[F1-ACTUATION-DONE] duration={self.total_duration_sec:g}s")
            self.destroy_timer(self.timer)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Fase1ActuacionPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, RCLError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
