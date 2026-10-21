"""Reference trajectories + horizon extraction (no ROS)."""
import numpy as np


def figure_eight(a=2.0, n=2000):
    t = np.linspace(0, 2 * np.pi, n)
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    return _with_heading(x, y)


def sharp_corners(side=3.0, n_per_side=300):
    """Square path: tests behaviour at 90 degree corners."""
    pts = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    xs, ys = [], []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        s = np.linspace(0, 1, n_per_side, endpoint=False)
        xs.append(x0 + s * (x1 - x0))
        ys.append(y0 + s * (y1 - y0))
    return _with_heading(np.concatenate(xs), np.concatenate(ys))


def _with_heading(x, y):
    th = np.arctan2(np.gradient(y), np.gradient(x))
    return np.vstack([x, y, th])  # (3, M)


class HorizonSampler:
    """Turns a geometric path into a time-parameterised reference for MPC.

    Finds the closest path point to the robot (searching forward only, so it
    never jumps back on self-intersecting paths like the figure eight), then
    samples N+1 points spaced by v_ref*dt along arc length."""

    def __init__(self, path, v_ref, dt, N, search_window=200):
        self.path, self.v_ref, self.dt, self.N = path, v_ref, dt, N
        d = np.linalg.norm(np.diff(path[:2], axis=1), axis=0)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.idx = 0
        self.win = search_window

    def nearest(self, xy):
        lo, hi = self.idx, min(self.idx + self.win, self.path.shape[1])
        d = np.linalg.norm(self.path[:2, lo:hi] - np.asarray(xy)[:, None], axis=0)
        self.idx = lo + int(np.argmin(d))
        return self.idx, float(d.min())

    def horizon(self, xy):
        i, _ = self.nearest(xy)
        s_target = self.s[i] + self.v_ref * self.dt * np.arange(self.N + 1)
        s_target = np.clip(s_target, 0, self.s[-1])
        idx = np.searchsorted(self.s, s_target).clip(0, self.path.shape[1] - 1)
        xref = self.path[:, idx]
        # feed-forward inputs from the reference itself
        dth = np.diff(np.unwrap(xref[2]))
        uref = np.vstack([np.full(self.N, self.v_ref), dth / self.dt])
        if s_target[-1] >= self.s[-1]:
            uref[0, s_target[:-1] >= self.s[-1]] = 0.0  # stop at the end
        return xref, uref

    def done(self, xy, tol=0.1):
        return self.idx >= self.path.shape[1] - 5 and \
            np.linalg.norm(self.path[:2, -1] - np.asarray(xy)) < tol


def cross_track_error(path, xy):
    return float(np.min(np.linalg.norm(path[:2] - np.asarray(xy)[:, None], axis=0)))
