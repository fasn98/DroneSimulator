"""
6-DoF helicopter: rigid body + rotor speed + tip-path-plane flapping + two turboshafts.

State vector:
    [0:3]   position, world ENU (m); z = height of the CG above the heliport
    [3:6]   velocity, world (m/s)
    [6:10]  attitude quaternion (body: x forward, y left, z up)
    [10:13] angular rate, body (rad/s)
    [13]    rotor speed Omega (rad/s)
    [14]    longitudinal tip-path-plane tilt beta_lon (rad, + = forward)
    [15]    lateral tip-path-plane tilt beta_lat (rad, + = to the right)
    [16:18] power delivered by engine 1 and 2 (W)
    [18]    fuel (kg)

Rotor speed (base of autorotation):   I Omega dOmega/dt = eta (P1 + P2) - P_main - P_tail - P_acc
Flapping (quasi-static, first order): dbeta/dt = (beta_cmd - beta)/tau_f - q   (tau_f = 16/(gamma Omega))
Engines (first order):                dP_i/dt = (P_cmd_i - P_i)/tau_e
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..dynamics import cross3, quat_derivative, quat_from_euler, quat_to_rot
from .isa import HeliAtmosphere
from .params import HeliParams
from .rotor import RotorState, main_rotor, tail_rotor_max_thrust, tail_rotor_power

N_STATE = 19


@dataclass
class Controls:
    collective: float = 0.0  # theta_75, rad
    cyc_lon: float = 0.0  # commanded forward disk tilt, rad
    cyc_lat: float = 0.0  # commanded right disk tilt, rad
    pedal: float = 0.0  # tail-rotor thrust fraction, -1..1 (+ = thrust to the left)
    p_cmd: tuple = (0.0, 0.0)  # engine power demand, W (from the governor)


@dataclass
class Aero:
    """Derived quantities at a state (telemetry, governor, SADPF)."""
    rho: float
    rotor: RotorState
    tail_thrust: float
    p_tail: float
    p_acc: float
    airspeed: float
    z_hub: float
    on_ground: bool


class HelicopterDynamics:
    GROUND_DEFLECTION = 0.02  # m of skid deflection at static load
    GROUND_DAMPING_RATIO = 0.9
    GROUND_FRICTION = 0.5

    def __init__(self, params: HeliParams, atmosphere: HeliAtmosphere):
        self.p = params
        self.atm = atmosphere
        self.inertia = np.diag(params.inertia)
        self.inertia_inv = np.linalg.inv(self.inertia)
        w, l, h = params.skid_w, params.skid_l, params.cg_h
        self.feet = [np.array([l, w, -h]), np.array([l, -w, -h]), np.array([-l, w, -h]), np.array([-l, -w, -h])]
        self.wind = np.zeros(3)
        self.engine_failed = [False, False]
        self._lam_i = 0.05  # warm start of the inflow iteration (converged value, not a state)
        self.fuel0 = params.fuel

    def mass(self, x: np.ndarray) -> float:
        return self.p.mass - (self.fuel0 - max(x[18], 0.0))

    def initial_state(self, position=(0.0, 0.0, None), yaw: float = 0.0, velocity=(0.0, 0.0, 0.0),
                      pitch: float = 0.0, engine_power_w: float = 0.0) -> np.ndarray:
        x = np.zeros(N_STATE)
        z = position[2] if position[2] is not None else self.p.cg_h - self.GROUND_DEFLECTION
        x[0:3] = [position[0], position[1], z]
        x[3:6] = velocity
        x[6:10] = quat_from_euler(0.0, pitch, yaw)
        x[13] = self.p.omega100
        x[16] = x[17] = engine_power_w / 2.0
        x[18] = self.fuel0
        return x

    # ------------------------------------------------------------------------------------------
    def aero(self, x: np.ndarray, u: Controls) -> tuple:
        p = self.p
        pos, vel, q = x[0:3], x[3:6], x[6:10]
        R = quat_to_rot(q / np.linalg.norm(q))
        omega = max(x[13], 1.0)
        rho = self.atm.density(pos[2])
        v_rel_b = R.T @ (vel - self.wind)
        bl, bt = x[14], x[15]
        n = np.array([math.sin(bl) * math.cos(bt), -math.sin(bt), math.cos(bl) * math.cos(bt)])
        v_tpp = float(v_rel_b @ n)
        v_in = float(np.linalg.norm(v_rel_b - v_tpp * n))
        z_hub = pos[2] + p.hub_h
        rs = main_rotor(p, u.collective, v_tpp, v_in, rho, omega, z_hub, lam_i0=self._lam_i, iters=12)
        self._lam_i = rs.lam_i
        t_tr = float(np.clip(u.pedal, -1.0, 1.0)) * tail_rotor_max_thrust(p, rho, omega)
        p_tr = tail_rotor_power(p, t_tr, rho, omega)
        on_ground = any((pos + R @ f)[2] < 0.0 for f in self.feet)
        return R, n, v_rel_b, Aero(rho, rs, t_tr, p_tr, p.p_acc_kw * 1e3, float(np.linalg.norm(v_rel_b)), z_hub,
                                   on_ground)

    def derivative(self, x: np.ndarray, u: Controls) -> np.ndarray:
        p = self.p
        pos, vel, w = x[0:3], x[3:6], x[10:13]
        R, n, v_rel_b, a = self.aero(x, u)
        m = self.mass(x)
        omega = max(x[13], 1.0)
        rs = a.rotor
        T = rs.thrust
        # fuselage download in the rotor wake, fading with speed
        vh = max(rs.v_h, 1e-3)
        dl = p.download / (1.0 + (a.airspeed / vh) ** 2)
        f_b = T * n + np.array([0.0, a.tail_thrust, -dl * max(T, 0.0)])
        # anisotropic fuselage drag (body axes)
        fx, fy, fz = p.f_drag, p.f_side, p.f_vertical
        f_b += -0.5 * a.rho * np.array([fx * abs(v_rel_b[0]) * v_rel_b[0], fy * abs(v_rel_b[1]) * v_rel_b[1],
                                        fz * abs(v_rel_b[2]) * v_rel_b[2]])
        # moments: rotor thrust at the hub, hub stiffness, tail rotor, rotor drive torque reaction, yaw damping
        r_hub = np.array([0.0, 0.0, p.hub_h])
        m_b = cross3(r_hub, T * n) + p.hub_k * np.array([x[15], x[14], 0.0])
        m_b += cross3(np.array([-p.tr_arm, 0.0, 0.0]), np.array([0.0, a.tail_thrust, 0.0]))
        p_eng = p.eta_tr * (max(x[16], 0.0) + max(x[17], 0.0))
        q_shaft = max(p_eng - a.p_tail - a.p_acc, 0.0) / omega
        m_b[2] += -p.rotor_dir * q_shaft - p.yaw_damping * w[2]

        force_w = R @ f_b + np.array([0.0, 0.0, -m * self.atm.gravity])
        # skids: spring-damper with Coulomb-capped friction
        k = m * self.atm.gravity / (len(self.feet) * self.GROUND_DEFLECTION)
        c = 2.0 * self.GROUND_DAMPING_RATIO * math.sqrt(k * m / len(self.feet))
        for foot in self.feet:
            pf = pos + R @ foot
            if pf[2] < 0.0:
                vf = vel + R @ cross3(w, foot)
                fzg = max(k * (-pf[2]) - c * vf[2], 0.0)
                ft = -c * vf[:2]
                cap = self.GROUND_FRICTION * fzg
                nt = np.linalg.norm(ft)
                if nt > cap and nt > 0:
                    ft *= cap / nt
                fg = np.array([ft[0], ft[1], fzg])
                force_w += fg
                m_b += cross3(foot, R.T @ fg)

        dx = np.zeros(N_STATE)
        dx[0:3] = vel
        dx[3:6] = force_w / m
        dx[6:10] = quat_derivative(x[6:10], w)
        dx[10:13] = self.inertia_inv @ (m_b - cross3(w, self.inertia @ w))
        dx[13] = (p_eng - rs.power - a.p_tail - a.p_acc) / (p.I_rotor * omega)
        tau_f = 16.0 / (p.lock * omega)
        dx[14] = (u.cyc_lon - x[14]) / tau_f - w[1]
        dx[15] = (u.cyc_lat - x[15]) / tau_f - w[0]
        for i in range(2):
            target = 0.0 if self.engine_failed[i] else u.p_cmd[i]
            tau = p.eng_fail_tau if self.engine_failed[i] else p.eng_tau
            dx[16 + i] = (target - x[16 + i]) / tau
        dx[18] = -p.sfc * (max(x[16], 0.0) + max(x[17], 0.0)) / 3.6e6 if x[18] > 0 else 0.0
        return dx

    def rk4_step(self, x: np.ndarray, u: Controls, dt: float) -> np.ndarray:
        k1 = self.derivative(x, u)
        k2 = self.derivative(x + 0.5 * dt * k1, u)
        k3 = self.derivative(x + 0.5 * dt * k2, u)
        k4 = self.derivative(x + dt * k3, u)
        xn = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        xn[6:10] /= np.linalg.norm(xn[6:10])
        xn[18] = max(xn[18], 0.0)
        return xn


def engine_limit_w(p: HeliParams, atm: HeliAtmosphere, z: float, rating: str, n_running: int) -> float:
    """Max power per running engine (W): thermodynamic rating with altitude lapse, capped by the
    transmission torque limit of the rating (shared between running engines)."""
    from .isa import RHO0
    sigma = atm.density(z) / RHO0
    thermo = p.engine_rating_kw(rating) * sigma ** p.lapse_exp
    gearbox = p.gearbox_limit_kw(rating) / max(n_running, 1)
    return 1e3 * min(thermo, gearbox)


def rating_for(n_running: int, t_since_failure: Optional[float]) -> str:
    """AEO: take-off rating. OEI: 30 s, then 2 min, then continuous (automatic, no pilot selection)."""
    if n_running == 2 or t_since_failure is None:
        return "TO"
    if t_since_failure < 30.0:
        return "OEI30"
    if t_since_failure < 150.0:
        return "OEI2"
    return "OEIC"
