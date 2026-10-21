"""Logs cross-track error, commands and solve time to CSV for the post's plots."""
import csv
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from mpc_diffdrive.reference import (HorizonSampler, cross_track_error,
                                     figure_eight, sharp_corners)


class Metrics(Node):
    def __init__(self):
        super().__init__("metrics_logger")
        name = self.declare_parameter("path", "figure_eight").value
        tag = self.declare_parameter("tag", "mpc").value
        out_dir = self.declare_parameter("out_dir", os.path.expanduser("~/mpc_logs")).value
        os.makedirs(out_dir, exist_ok=True)
        self.path = figure_eight() if name == "figure_eight" else sharp_corners()
        # separate sampler just to know when the lap is finished
        self.progress = HorizonSampler(self.path, 0.35, 0.1, 1)
        self.timeout = self.declare_parameter("timeout", 120.0).value
        self.t_start = None
        self.finished = False
        fn = os.path.join(out_dir, f"{tag}_{name}.csv")
        self.f = open(fn, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "x", "y", "cte", "v_cmd", "w_cmd", "solve_ms"])
        self.cmd = (0.0, 0.0)
        self.solve_ms = float("nan")
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 20)
        self.create_subscription(Float64, "mpc/solve_time_ms", self.on_t, 20)
        self.get_logger().info(f"Logging to {fn}")

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_t(self, m):
        self.solve_ms = m.data

    def on_odom(self, m):
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.finished:
            return
        if self.t_start is None:
            self.t_start = t
        cte = cross_track_error(self.path, (p.x, p.y))
        self.w.writerow([f"{t - self.t_start:.3f}", p.x, p.y, cte, *self.cmd, self.solve_ms])
        self.progress.nearest((p.x, p.y))
        if self.progress.done((p.x, p.y), tol=0.15) or t - self.t_start > self.timeout:
            self.finished = True
            self.f.flush()
            self.get_logger().info(f"Lap finished after {t - self.t_start:.1f} s sim time")
            raise SystemExit  # ends spin, launch file shuts everything down

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    n = Metrics()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
