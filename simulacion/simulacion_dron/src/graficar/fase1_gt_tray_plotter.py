#!/usr/bin/env python3
import threading
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Fase1GtTrayPlotter(Node):
    def __init__(self):
        super().__init__("fase1_gt_tray_plotter")
        self.declare_parameter("input_topic", "graficas/gt_vs_tray")
        self.declare_parameter("window_seconds", 24.0)
        self.declare_parameter("max_samples", 2400)

        self.input_topic = self.get_parameter("input_topic").value
        self.window_seconds = float(self.get_parameter("window_seconds").value)
        self.max_samples = int(self.get_parameter("max_samples").value)
        self.lock = threading.Lock()
        self.start_time = None
        self.times = deque(maxlen=self.max_samples)
        self.tray = [deque(maxlen=self.max_samples) for _ in range(4)]
        self.gt = [deque(maxlen=self.max_samples) for _ in range(4)]
        self.redraw_pending = False
        self.subscription = self.create_subscription(
            Float64MultiArray, self.input_topic, self.on_sample, 10)

        self.figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        self.axes = list(axes.flat)
        labels = (("x", "m"), ("y", "m"), ("z", "m"), ("yaw", "rad"))
        self.lines = []
        for axis, (name, unit) in zip(self.axes, labels):
            tray_line, = axis.plot([], [], label="Tray", color="#1565c0")
            gt_line, = axis.plot([], [], label="GT", color="#e65100", linestyle="--")
            axis.set_title(name)
            axis.set_ylabel(unit)
            axis.grid(True)
            axis.legend(loc="upper right")
            self.lines.append((tray_line, gt_line))
        self.axes[2].set_xlabel("t de simulacion (s)")
        self.axes[3].set_xlabel("t de simulacion (s)")
        self.figure.suptitle("Fase 1 - GT frente a Tray")
        self.figure.tight_layout()

        self.get_logger().info(f"F1 GT/Tray plotter ready: input={self.input_topic}")

    def on_sample(self, message):
        if len(message.data) != 8:
            self.get_logger().warning("Ignored GT/Tray sample with invalid size")
            return
        now = self.get_clock().now()
        with self.lock:
            if self.start_time is None:
                self.start_time = now
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            self.times.append(elapsed)
            for index in range(4):
                self.tray[index].append(message.data[index])
                self.gt[index].append(message.data[index + 4])
            self.redraw_pending = True

    def redraw(self):
        with self.lock:
            if not self.redraw_pending or not self.times:
                return
            self.redraw_pending = False
            times = list(self.times)
            tray = [list(values) for values in self.tray]
            gt = [list(values) for values in self.gt]

        lower = max(0.0, times[-1] - self.window_seconds) if self.window_seconds > 0.0 else 0.0
        for index, axis in enumerate(self.axes):
            tray_line, gt_line = self.lines[index]
            tray_line.set_data(times, tray[index])
            gt_line.set_data(times, gt[index])
            axis.set_xlim(lower, max(times[-1], lower + 1.0))
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        self.figure.canvas.draw_idle()


def main():
    rclpy.init()
    node = Fase1GtTrayPlotter()

    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin_node, daemon=True)
    executor_thread.start()
    timer = node.figure.canvas.new_timer(interval=100)
    timer.add_callback(node.redraw)
    timer.start()
    try:
        plt.show()
    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
