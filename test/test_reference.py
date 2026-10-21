import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mpc_diffdrive.reference import (HorizonSampler, cross_track_error,  # noqa: E402
                                     figure_eight, sharp_corners)


def test_sampler_does_not_jump_lobes_at_crossing():
    path = figure_eight()
    s = HorizonSampler(path, 0.35, 0.1, 20)
    # walk along the path; index must never go backwards
    last = 0
    for i in range(0, path.shape[1], 25):
        idx, _ = s.nearest(path[:2, i])
        assert idx >= last
        last = idx
    assert last > path.shape[1] - 30


def test_horizon_shape_and_spacing():
    s = HorizonSampler(sharp_corners(), 0.35, 0.1, 20)
    xref, uref = s.horizon((0.0, 0.0))
    assert xref.shape == (3, 21) and uref.shape == (2, 20)
    step = np.linalg.norm(np.diff(xref[:2], axis=1), axis=0)
    assert np.allclose(step[:5], 0.035, atol=0.011)


def test_cte_zero_on_path():
    p = figure_eight()
    assert cross_track_error(p, p[:2, 500]) == 0.0
