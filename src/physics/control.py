"""
Flight control: geometric attitude controller (Lee et al., 2010) under a PID
position loop, plus a saturation-aware control allocator.

The allocator solves a bounded least-squares problem
    min || W (B u - w) ||   subject to  0 <= u <= u_max
so actuator limits (thin air, failed motor, empty tank) are respected and the
loss of control authority is visible instead of silently ignored.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from .dynamics import RigidBodyDynamics, cross3, quat_to_rot


def _vee(m: np.ndarray) -> np.ndarray:
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


@dataclass
class Gains:
    pos_wn: float = 0.9  # rad/s
    pos_zeta: float = 0.9
    pos_ki: float = 0.08
    att_wn: float = 4.0  # rad/s
    att_zeta: float = 0.85
    max_tilt_deg: float = 25.0
    max_climb_accel: float = 1.5  # m/s^2 above hover
    max_descent_accel: float = 1.0  # m/s^2 below hover


class Allocator:
    def __init__(self, dyn: RigidBodyDynamics):
        self.dyn = dyn
        v = dyn.vehicle
        self.arm = max([np.linalg.norm(r.position[:2]) for r in v.rotors] +
                       [np.linalg.norm(t.position[:2]) for t in v.thrusters] + [0.1])

    def matrix(self, x: np.ndarray) -> np.ndarray:
        dyn, v = self.dyn, self.dyn.vehicle
        alt = x[2]
        rho, a = dyn.body.density(alt), dyn.body.speed_of_sound(alt)
        n_lift = max(len(v.rotors), 1)
        t_hover = dyn.mass(x) * dyn.body.gravity / n_lift
        cols = []
        for r in v.rotors:
            kq = r.torque(t_hover, rho, a) / t_hover if t_hover > 0 else 0.0
            cols.append([1.0, r.position[1], -r.position[0], -r.spin * kq])
        for t in v.thrusters:
            m = cross3(t.position, t.direction)
            cols.append([t.direction[2], m[0], m[1], m[2]])
        return np.array(cols).T

    def solve(self, x: np.ndarray, wrench: np.ndarray, limits: np.ndarray) -> np.ndarray:
        if limits.max() <= 1e-9:
            return np.zeros_like(limits)
        B = self.matrix(x)
        f_ref = self.dyn.mass(x) * self.dyn.body.gravity
        m_ref = f_ref * self.arm
        W = np.diag([1.0 / f_ref, 3.0 / m_ref, 3.0 / m_ref, 0.3 / m_ref])
        ub = np.maximum(limits, 1e-9)
        res = lsq_linear(W @ B, W @ wrench, bounds=(np.zeros_like(ub), ub), method="bvls")
        return res.x


class GeometricController:
    def __init__(self, dyn: RigidBodyDynamics, gains: Gains | None = None):
        self.dyn = dyn
        self.g = gains or Gains()
        self.alloc = Allocator(dyn)
        self.integral = np.zeros(3)
        self.last_wrench = np.zeros(4)

    def reset(self):
        self.integral[:] = 0.0

    def update(self, x: np.ndarray, pos_ref: np.ndarray, vel_ref: np.ndarray, yaw_ref: float,
               dt: float) -> np.ndarray:
        g, dyn = self.g, self.dyn
        grav = dyn.body.gravity
        m = dyn.mass(x)
        p, v, q, w = x[0:3], x[3:6], x[6:10], x[10:13]
        R = quat_to_rot(q)

        # --- position loop --------------------------------------------------
        kp, kd = g.pos_wn ** 2, 2 * g.pos_zeta * g.pos_wn
        e_p, e_v = p - pos_ref, v - vel_ref
        self.integral = np.clip(self.integral + e_p * dt, -2.0, 2.0)
        acc = -kp * e_p - kd * e_v - g.pos_ki * self.integral
        h_max = grav * np.tan(np.radians(g.max_tilt_deg))
        h = np.linalg.norm(acc[:2])
        if h > h_max:
            acc[:2] *= h_max / h
        acc[2] = np.clip(acc[2], -g.max_descent_accel, g.max_climb_accel)
        f_des = m * (acc + np.array([0.0, 0.0, grav]))
        f_des[2] = max(f_des[2], 0.05 * m * grav)

        # --- attitude loop (SO(3)) ----------------------------------------
        b3 = f_des / np.linalg.norm(f_des)
        b1c = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])
        b2 = cross3(b3, b1c)
        b2 /= np.linalg.norm(b2)
        b1 = cross3(b2, b3)
        R_des = np.column_stack([b1, b2, b3])
        thrust = float(f_des @ R[:, 2])
        e_R = 0.5 * _vee(R_des.T @ R - R.T @ R_des)
        I = dyn.inertia
        K_R = I * g.att_wn ** 2
        K_w = I * 2 * g.att_zeta * g.att_wn
        moment = -K_R @ e_R - K_w @ w + cross3(w, I @ w)

        wrench = np.array([max(thrust, 0.0), *moment])
        self.last_wrench = wrench
        return self.alloc.solve(x, wrench, dyn.actuator_limits(x))
