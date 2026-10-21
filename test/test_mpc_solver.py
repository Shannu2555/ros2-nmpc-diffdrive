import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from mpc_diffdrive.mpc_solver import DiffDriveMPC, MPCParams  # noqa: E402


def test_respects_input_bounds():
    p = MPCParams()
    mpc = DiffDriveMPC(p)
    xref = np.zeros((3, p.N + 1)); xref[0] = 5.0          # far-away target
    u0, _, U, st = mpc.solve(np.zeros(3), np.zeros(2), xref, np.zeros((2, p.N)))
    assert st["success"]
    assert np.all(U[0] <= p.v_max + 1e-6) and np.all(U[1] <= p.w_max + 1e-6)
    assert abs(u0[0]) <= p.a_max * p.dt + 1e-6             # rate limit from rest


def test_closed_loop_tracks_figure_eight():
    from offline_sim import run
    _, L = run("figure_eight", noise=0.0, steps=400)
    assert np.sqrt(np.mean(L[:, 5] ** 2)) < 0.1
