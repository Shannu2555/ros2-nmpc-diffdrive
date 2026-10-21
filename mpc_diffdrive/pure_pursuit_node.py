"""Baseline: pure pursuit on the same path, for a fair comparison with MPC.
Same topics as mpc_node so the metrics logger works unchanged."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mpc_diffdrive.mpc_node import yaw_from_quat
from mpc_diffdrive.reference import HorizonSampler, figure_eight, sharp_corners


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.Ld = self.declare_parameter("lookahead", 0.4).value
        self.v = self.declare_parameter("v_ref", 0.35).value
        self.w_max = self.declare_parameter("w_max", 1.5).value
        name = self.declare_parameter("path", "figure_eight").value
        self.path = figure_eight() if name == "figure_eight" else sharp_corners()
        self.sampler = HorizonSampler(self.path, self.v, 0.1, 1)
        self.state = None
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_timer(0.1, self.step)

    def on_odom(self, m):
        p = m.pose.pose
        self.state = np.array([p.position.x, p.position.y, yaw_from_quat(p.orientation)])

    def step(self):
        if self.state is None:
            return
        x, y, th = self.state
        i, _ = self.sampler.nearest((x, y))
        s_goal = self.sampler.s[i] + self.Ld
        j = min(int(np.searchsorted(self.sampler.s, s_goal)), self.path.shape[1] - 1)
        gx, gy = self.path[0, j], self.path[1, j]
        # lookahead point in robot frame
        dx, dy = gx - x, gy - y
        ly = -math.sin(th) * dx + math.cos(th) * dy
        curvature = 2.0 * ly / max(self.Ld ** 2, 1e-6)
        cmd = Twist()
        cmd.linear.x = self.v if not self.sampler.done((x, y)) else 0.0
        cmd.angular.z = float(np.clip(self.v * curvature, -self.w_max, self.w_max))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
