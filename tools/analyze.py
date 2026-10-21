"""Turn the CSVs from metrics_node into plots + a markdown table.

  python3 tools/analyze.py <log_dir> <out_dir>
"""
import os
import platform
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mpc_diffdrive.reference import figure_eight, sharp_corners  # noqa: E402

PATHS = {"figure_eight": figure_eight(), "sharp_corners": sharp_corners()}
CTRLS = {"mpc": ("NMPC", "tab:orange"), "pure_pursuit": ("Pure pursuit", "tab:blue")}


def load(fn):
    d = np.genfromtxt(fn, delimiter=",", names=True)
    return d


def solve_times(d):
    s = d["solve_ms"]
    s = s[~np.isnan(s)]
    if len(s) == 0:
        return s
    # odom is 50 Hz, MPC is 10 Hz, so each solve shows up ~5 times in a row
    keep = np.concatenate([[True], np.diff(s) != 0])
    return s[keep]


def cpu_name():
    try:
        for line in open("/proc/cpuinfo"):
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def main(log_dir, out):
    os.makedirs(out, exist_ok=True)
    rows = []
    data = {}
    for pname in PATHS:
        for c in CTRLS:
            fn = os.path.join(log_dir, f"{c}_{pname}.csv")
            if os.path.exists(fn):
                data[(c, pname)] = load(fn)

    for pname, path in PATHS.items():
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
        ax[0].plot(path[0], path[1], "k--", lw=1, label="reference")
        for c, (label, col) in CTRLS.items():
            d = data.get((c, pname))
            if d is None:
                continue
            ax[0].plot(d["x"], d["y"], color=col, lw=1.8, label=label)
            ax[1].plot(d["t"], d["cte"] * 100, color=col, lw=1.2, label=label)
            st = solve_times(d)
            rows.append([pname, label, np.sqrt(np.mean(d["cte"] ** 2)) * 100,
                         d["cte"].max() * 100, d["t"][-1],
                         np.median(st) if len(st) else None,
                         np.percentile(st, 95) if len(st) else None,
                         st.max() if len(st) else None])
        ax[0].axis("equal"); ax[0].legend(); ax[0].set_title(f"{pname}: path (Gazebo odom)")
        ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
        ax[1].set_title("cross-track error"); ax[1].set_xlabel("sim time [s]")
        ax[1].set_ylabel("error [cm]"); ax[1].legend(); ax[1].grid(alpha=.3)
        plt.tight_layout(); plt.savefig(os.path.join(out, f"{pname}_tracking.png"), dpi=130)
        plt.close()

    st = np.concatenate([solve_times(d) for (c, _), d in data.items() if c == "mpc"])
    plt.figure(figsize=(6.5, 4))
    plt.hist(st, bins=40, color="tab:orange")
    plt.axvline(100, color="r", ls="--", label="control period (100 ms)")
    plt.axvline(np.median(st), color="k", ls=":", label=f"median {np.median(st):.1f} ms")
    plt.xlabel("NMPC solve time [ms]"); plt.ylabel("count"); plt.legend()
    plt.title(f"IPOPT solve time, both paths ({len(st)} solves)")
    plt.tight_layout(); plt.savefig(os.path.join(out, "solve_time_hist.png"), dpi=130)
    plt.close()

    f = lambda v: "-" if v is None else f"{v:.2f}"  # noqa: E731
    md = ["| Path | Controller | CTE RMSE [cm] | CTE max [cm] | Lap time [s] | Solve median [ms] | Solve p95 [ms] | Solve max [ms] |",
          "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r[0]} | {r[1]} | {r[2]:.2f} | {r[3]:.2f} | {r[4]:.1f} | {f(r[5])} | {f(r[6])} | {f(r[7])} |")
    md.append("")
    md.append(f"CPU: {cpu_name()} ({os.cpu_count()} threads visible), N=20, dt=0.1 s, v_ref=0.35 m/s")
    open(os.path.join(out, "metrics.md"), "w").write("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
