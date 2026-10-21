"""ROS 2 Humble node: nonlinear MPC path tracker for a diff-drive robot.

Subscribes: /odom (nav_msgs/Odometry)
Publishes : /cmd_vel (geometry_msgs/Twist)
            /mpc/reference (nav_msgs/Path)   full reference, latched
            /mpc/predicted (nav_msgs/Path)   MPC predicted trajectory (RViz)
            /mpc/solve_time_ms (std_msgs/Float64)
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64

from mpc_diffdrive.mpc_solver import DiffDriveMPC, MPCParams, unwrap_reference
from mpc_diffdrive.reference import HorizonSampler, figure_eight, sharp_corners


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_controller")
        d = self.declare_parameter
        p = MPCParams(
            N=d("horizon", 20).value,
            dt=d("dt", 0.1).value,
            v_max=d("v_max", 0.5).value,
            v_min=d("v_min", -0.1).value,
            w_max=d("w_max", 1.5).value,
            a_max=d("a_max", 1.0).value,
            alpha_max=d("alpha_max", 3.0).value,
            Q=list(d("Q", [10.0, 10.0, 1.0]).value),
            R=list(d("R", [0.5, 0.1]).value),
            Rd=list(d("Rd", [1.0, 0.5]).value),
            P=list(d("P", [20.0, 20.0, 2.0]).value),
        )
        v_ref = d("v_ref", 0.35).value
        path_name = d("path", "figure_eight").value
        self.path = figure_eight() if path_name == "figure_eight" else sharp_corners()

        self.mpc = DiffDriveMPC(p)
        self.sampler = HorizonSampler(self.path, v_ref, p.dt, p.N)
        self.u_prev = np.zeros(2)
        self.state = None
        self.finished = False

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub = self.create_publisher(Path, "mpc/reference", latched)
        self.pred_pub = self.create_publisher(Path, "mpc/predicted", 10)
        self.t_pub = self.create_publisher(Float64, "mpc/solve_time_ms", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.timer = self.create_timer(p.dt, self.step)
        self.ref_pub.publish(self.to_path(self.path, "odom"))
        self.get_logger().info(f"MPC ready: N={p.N}, dt={p.dt}, path={path_name}")

    def on_odom(self, msg: Odometry):
        pp = msg.pose.pose
        self.state = np.array([pp.position.x, pp.position.y, yaw_from_quat(pp.orientation)])

    def step(self):
        if self.state is None or self.finished:
            return
        x = self.state.copy()
        if self.sampler.done(x[:2]):
            self.finished = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Reached end of path.")
            return
        xref, uref = self.sampler.horizon(x[:2])
        xref[2] = unwrap_reference(x[2], xref[2])

        t0 = time.perf_counter()
        u0, Xp, _, stats = self.mpc.solve(x, self.u_prev, xref, uref)
        dt_ms = (time.perf_counter() - t0) * 1e3
        self.t_pub.publish(Float64(data=dt_ms))

        if not stats["success"]:
            # IPOPT gave up: slow down instead of trusting a half-converged plan
            self.get_logger().warn(f"MPC solve failed (iter={stats['iter']}), braking")
            u0 = self.u_prev * 0.5
        if dt_ms > self.mpc.p.dt * 1e3:
            self.get_logger().warn(f"Solve {dt_ms:.1f} ms exceeds control period")

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(u0[0]), float(u0[1])
        self.cmd_pub.publish(cmd)
        self.u_prev = np.asarray(u0, dtype=float)
        self.pred_pub.publish(self.to_path(Xp, "odom"))

    def to_path(self, arr, frame):
        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for k in range(arr.shape[1]):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(arr[0, k]), float(arr[1, k])
            ps.pose.orientation.z = math.sin(arr[2, k] / 2)
            ps.pose.orientation.w = math.cos(arr[2, k] / 2)
            msg.poses.append(ps)
        return msg


def main():
    rclpy.init()
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # context is already gone after ctrl-c
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
