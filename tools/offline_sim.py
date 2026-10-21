"""ROS-free closed-loop simulation: tune the MPC here before touching Gazebo.

  python3 tools/offline_sim.py --path figure_eight --noise 0.01 --plot

Plant = same unicycle model + optional Gaussian noise on the measured state
and a first-order actuator lag (model mismatch on purpose, so the result is
not trivially perfect)."""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mpc_diffdrive.mpc_solver import DiffDriveMPC, MPCParams, unwrap_reference  # noqa: E402
from mpc_diffdrive.reference import (HorizonSampler, cross_track_error,  # noqa: E402
                                     figure_eight, sharp_corners)


def plant_step(x, u, dt):
    return x + dt * np.array([u[0] * np.cos(x[2]), u[0] * np.sin(x[2]), u[1]])


def run(path_name="figure_eight", noise=0.0, lag_tau=0.05, steps=1200, seed=0):
    rng = np.random.default_rng(seed)
    p = MPCParams()
    path = figure_eight() if path_name == "figure_eight" else sharp_corners()
    mpc, samp = DiffDriveMPC(p), HorizonSampler(path, 0.35, p.dt, p.N)
    x = path[:, 0].copy() + np.array([0.0, 0.1, 0.0])   # start slightly off-path
    u_applied, u_prev = np.zeros(2), np.zeros(2)
    log = []
    sub = 10  # plant substeps per control step
    for _ in range(steps):
        meas = x + rng.normal(0, noise, 3) * np.array([1, 1, 0.5])
        if samp.done(meas[:2]):
            break
        xref, uref = samp.horizon(meas[:2])
        xref[2] = unwrap_reference(meas[2], xref[2])
        u0, _, _, st = mpc.solve(meas, u_prev, xref, uref)
        u_prev = u0
        for _ in range(sub):  # actuator lag + integration
            u_applied += (p.dt / sub) / lag_tau * (u0 - u_applied) if lag_tau > 0 else (u0 - u_applied)
            x = plant_step(x, u_applied, p.dt / sub)
        log.append([*x, *u0, cross_track_error(path, x[:2]), st["t_wall"] * 1e3, st["success"]])
    return path, np.array(log)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="figure_eight", choices=["figure_eight", "sharp_corners"])
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    path, L = run(a.path, a.noise)
    cte, t = L[:, 5], L[:, 6]
    print(f"steps={len(L)}  CTE rmse={np.sqrt(np.mean(cte**2)):.4f} m  max={cte.max():.4f} m")
    print(f"solve ms: median={np.median(t):.2f}  p95={np.percentile(t, 95):.2f}  max={t.max():.2f}")
    print(f"failed solves: {int((L[:, 7] == 0).sum())}")
    if a.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
        ax[0].plot(path[0], path[1], "k--", lw=1, label="reference")
        ax[0].plot(L[:, 0], L[:, 1], lw=2, label="MPC")
        ax[0].axis("equal"); ax[0].legend(); ax[0].set_title("Path")
        ax[1].plot(cte); ax[1].set_title("Cross-track error [m]"); ax[1].set_xlabel("step")
        plt.tight_layout(); plt.savefig(f"offline_{a.path}.png", dpi=150); plt.show()
