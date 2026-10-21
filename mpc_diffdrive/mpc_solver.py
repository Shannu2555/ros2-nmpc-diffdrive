"""Nonlinear MPC for a differential-drive (unicycle) robot.

Pure Python + CasADi, no ROS imports, so it can be unit-tested and
benchmarked offline (see tools/offline_sim.py and test/).

State     x = [px, py, theta]
Control   u = [v, omega]
Model     px' = v cos(theta), py' = v sin(theta), theta' = omega
Discretization: RK4 with step dt.
Transcription: multiple shooting (states and controls are both decision
variables, dynamics enforced as equality constraints). This is usually
better conditioned than single shooting for nonlinear models.

Cost:
  sum_k ||x_k - xref_k||_Q^2 + ||u_k - uref_k||_R^2 + ||u_k - u_{k-1}||_Rd^2
  + ||x_N - xref_N||_P^2
Constraints:
  |v| <= v_max, |omega| <= w_max, |dv| <= a_max*dt, |domega| <= alpha_max*dt
"""
from dataclasses import dataclass, field

import time

import casadi as ca
import numpy as np


@dataclass
class MPCParams:
    N: int = 20                 # horizon steps
    dt: float = 0.1             # [s] must match the control loop period
    v_max: float = 0.5          # [m/s]
    v_min: float = -0.1         # allow a little reverse
    w_max: float = 1.5          # [rad/s]
    a_max: float = 1.0          # [m/s^2]
    alpha_max: float = 3.0      # [rad/s^2]
    Q: list = field(default_factory=lambda: [10.0, 10.0, 1.0])
    R: list = field(default_factory=lambda: [0.5, 0.1])
    Rd: list = field(default_factory=lambda: [1.0, 0.5])
    P: list = field(default_factory=lambda: [20.0, 20.0, 2.0])
    ipopt_max_iter: int = 100
    ipopt_print_level: int = 0


def unicycle_rhs(x, u):
    return ca.vertcat(u[0] * ca.cos(x[2]), u[0] * ca.sin(x[2]), u[1])


def rk4(f, x, u, dt):
    k1 = f(x, u)
    k2 = f(x + dt / 2 * k1, u)
    k3 = f(x + dt / 2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


class DiffDriveMPC:
    NX, NU = 3, 2

    def __init__(self, p: MPCParams):
        self.p = p
        self._build()
        self._x_guess = None
        self._u_guess = None

    def _build(self):
        p, NX, NU, N = self.p, self.NX, self.NU, self.p.N
        opti = ca.Opti()
        X = opti.variable(NX, N + 1)
        U = opti.variable(NU, N)
        x0 = opti.parameter(NX)           # measured state
        u_prev = opti.parameter(NU)       # last applied command
        Xref = opti.parameter(NX, N + 1)  # reference states
        Uref = opti.parameter(NU, N)      # reference (feed-forward) inputs

        Q, R, Rd, P = (ca.diag(ca.DM(v)) for v in (p.Q, p.R, p.Rd, p.P))
        cost = 0
        opti.subject_to(X[:, 0] == x0)
        for k in range(N):
            e = X[:, k] - Xref[:, k]
            du = U[:, k] - (u_prev if k == 0 else U[:, k - 1])
            cost += ca.mtimes([e.T, Q, e])
            cost += ca.mtimes([(U[:, k] - Uref[:, k]).T, R, U[:, k] - Uref[:, k]])
            cost += ca.mtimes([du.T, Rd, du])
            opti.subject_to(X[:, k + 1] == rk4(unicycle_rhs, X[:, k], U[:, k], p.dt))
            # input bounds
            opti.subject_to(opti.bounded(p.v_min, U[0, k], p.v_max))
            opti.subject_to(opti.bounded(-p.w_max, U[1, k], p.w_max))
            # rate (acceleration) bounds
            opti.subject_to(opti.bounded(-p.a_max * p.dt, du[0], p.a_max * p.dt))
            opti.subject_to(opti.bounded(-p.alpha_max * p.dt, du[1], p.alpha_max * p.dt))
        eN = X[:, N] - Xref[:, N]
        cost += ca.mtimes([eN.T, P, eN])
        opti.minimize(cost)
        opti.solver("ipopt",
                    {"expand": True, "print_time": False},
                    {"max_iter": p.ipopt_max_iter,
                     "print_level": p.ipopt_print_level,
                     "warm_start_init_point": "yes",
                     "sb": "yes"})
        self.opti, self.X, self.U = opti, X, U
        self.x0, self.u_prev, self.Xref, self.Uref = x0, u_prev, Xref, Uref

    def solve(self, x0, u_prev, xref, uref):
        """x0 (3,), u_prev (2,), xref (3, N+1), uref (2, N).

        Returns (u0, X_pred, U_pred, stats). xref[2] must already be
        unwrapped relative to x0[2] (see unwrap_reference)."""
        o = self.opti
        o.set_value(self.x0, x0)
        o.set_value(self.u_prev, u_prev)
        o.set_value(self.Xref, xref)
        o.set_value(self.Uref, uref)
        # warm start: shift the previous solution by one step
        if self._x_guess is not None:
            o.set_initial(self.X, np.hstack([self._x_guess[:, 1:], self._x_guess[:, -1:]]))
            o.set_initial(self.U, np.hstack([self._u_guess[:, 1:], self._u_guess[:, -1:]]))
        else:
            o.set_initial(self.X, xref)
            o.set_initial(self.U, uref)
        t0 = time.perf_counter()
        try:
            sol = o.solve()
            Xs, Us = sol.value(self.X), sol.value(self.U)
            ok = True
        except RuntimeError:
            # Solver failed (infeasible / max_iter). Use the debug values but
            # flag it; caller should decide on a safe fallback.
            Xs, Us = o.debug.value(self.X), o.debug.value(self.U)
            ok = False
        t_wall = time.perf_counter() - t0
        Us = np.atleast_2d(Us).reshape(self.NU, -1)
        self._x_guess, self._u_guess = Xs, Us
        stats = o.stats()
        return Us[:, 0], Xs, Us, {"success": ok,
                                  "iter": stats.get("iter_count", -1),
                                  "t_wall": t_wall}


def unwrap_reference(theta_robot, theta_ref):
    """Shift reference headings so they are continuous and start within
    +-pi of the robot heading. Avoids the 2*pi jump making the cost explode."""
    seq = np.unwrap(np.concatenate([[theta_robot], theta_ref]))
    return seq[1:]
