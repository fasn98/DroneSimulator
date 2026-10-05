"""
Helicopter simulator on the shared SimulationLoop: physics 200 Hz, SAS/autopilot + governor 50 Hz.

Guidance functions are the same as the drone's (`hover_at`, `waypoint_route` return a
(position, velocity, yaw) setpoint), so scenarios reuse the existing route code.

SAS / autopilot (simple, for scripted scenarios)
    velocity/position loop -> desired thrust vector -> desired attitude (SO(3) error as in the drone)
    attitude loop -> moments -> tip-path-plane tilt  beta = M / (K_hub + h T)
    vertical loop -> required thrust -> collective from the blade-element relation at the current inflow
    heading loop -> yaw moment -> tail-rotor thrust (balancing the main-rotor drive torque)
    autorotation mode: collective holds rotor speed (NR) instead of height; height is free
Governor: engine power demand = rotor + tail + accessories (+ NR error term), split between running
engines and limited by each engine's rating (AEO take-off; OEI 30 s / 2 min / continuous).
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

import numpy as np

from ..control import _vee
from ..dynamics import euler_from_quat, quat_to_rot
from ..simloop import SimulationLoop, Telemetry, WindModel
from .isa import HeliAtmosphere, RHO0
from .model import Controls, HelicopterDynamics, Terrain, engine_limit_w, rating_for
from .params import HeliParams
from .rotor import calibrate_drag_area, fin_side_force, fuel_flow_params, tail_rotor_max_thrust

KT = 0.514444


@dataclass
class EngineFault:
    """Engine `index` (0 or 1) flames out at t_start."""
    index: int
    t_start: float


@dataclass
class SasGains:
    pos_wn: float = 0.35
    pos_zeta: float = 1.0
    vel_ki: float = 0.05
    vz_tau: float = 1.2  # s, vertical speed response
    vz_ki: float = 0.15
    att_wn: float = 2.5
    att_zeta: float = 0.8
    yaw_wn: float = 1.5
    yaw_zeta: float = 0.9
    max_tilt_deg: float = 20.0
    max_vz: float = 5.0  # m/s climb / descent commanded by the position loop
    max_speed: float = 70.0  # m/s horizontal
    gov_tau: float = 0.6  # s, governor NR recovery
    nr_kp: float = 0.8  # rad of collective per unit NR error (autorotation)
    nr_ki: float = 0.6
    autorotation_collective: float = math.radians(3.0)  # collective lowered to this, then NR loop trims it


COMP_BAND = 0.25  # fraction of its share an engine may lag before the other compensates (ESTIMADO)


COLLECTIVE_LOWER_RATE = math.radians(1.5)  # rad/s, collective lowered after touchdown (ESTIMADO)


class HelicopterSimulator(SimulationLoop):
    def __init__(self, params: Optional[HeliParams] = None, atmosphere: Optional[HeliAtmosphere] = None,
                 dt: float = 0.005, control_rate_hz: float = 50.0, seed: int = 0, wind: bool = True,
                 engine_faults: Sequence[EngineFault] = (), gains: Optional[SasGains] = None,
                 start_position=(0.0, 0.0, None), start_velocity=(0.0, 0.0, 0.0), start_yaw: float = 0.0,
                 terrain: Optional[Terrain] = None, sadpf: bool = False):
        self.p = copy.deepcopy(params) if params is not None else HeliParams()
        if self.p.f_drag <= 0.0:
            self.p.f_drag = calibrate_drag_area(self.p)
        if self.p.ff_idle_kgh <= 0.0:
            self.p.ff_idle_kgh, self.p.sfc_marginal = fuel_flow_params(self.p)
        self.atm = atmosphere or HeliAtmosphere()
        self.dt = dt
        self.g = gains or SasGains()
        self.dyn = HelicopterDynamics(self.p, self.atm, terrain)
        self.ctrl_every = max(int(round(1.0 / (control_rate_hz * dt))), 1)
        self.rng = np.random.default_rng(seed)
        self.wind = WindModel(self.atm.as_body(), self.rng) if wind else None
        airborne = start_position[2] is not None
        from .rotor import level_flight, main_rotor
        if airborne:
            hover_w = level_flight(self.p, float(np.linalg.norm(start_velocity)), self.atm.density(start_position[2])
                                   ).p_engines
        else:
            # on the skids, engines at flight idle with flat collective (2 deg): rotor profile + tail + accessories
            rs0 = main_rotor(self.p, math.radians(2.0), 0.0, 0.0, self.atm.density(0.0), self.p.omega100,
                             self.p.cg_h + self.p.hub_h)
            hover_w = (rs0.power + self.p.p_acc_kw * 1e3) / self.p.eta_tr
        self.x = self.dyn.initial_state(start_position, start_yaw, start_velocity, engine_power_w=hover_w)
        self.t = 0.0
        self.telemetry = Telemetry()
        self._k = 0
        self.faults = list(engine_faults)
        self.fail_time: Optional[float] = None
        self.autorotation = False
        self.p_need_w = 0.0  # governor power demand (W), read by the SADPF
        self.nr_ref = 1.0  # NR reference of the autorotation collective loop (fraction of 100 %)
        self.collective_hold: Optional[float] = None  # set to freeze the collective (no pilot action)
        theta0 = math.radians(2.0)
        if airborne:  # start trimmed: collective for the level-flight thrust at that speed
            from .rotor import collective_for_thrust
            tp = level_flight(self.p, float(np.linalg.norm(start_velocity)), self.atm.density(start_position[2]))
            V = float(np.linalg.norm(start_velocity))
            a = math.radians(tp.alpha_deg)
            theta0 = collective_for_thrust(self.p, tp.thrust, V * math.sin(a), V * math.cos(a),
                                           self.atm.density(start_position[2]), self.p.omega100)
        self.u = Controls(collective=theta0, p_cmd=(hover_w / 2.0, hover_w / 2.0))
        self._vel_int = np.zeros(2)
        self._vz_int = 0.0
        self._nr_int = 0.0
        self._last_aero = None
        self.rating = "TO"
        self.events: List[dict] = []
        self.cushion = False  # collective follows the vertical-speed loop with no power (flare / landing)
        self.flight_phase: Optional[str] = None  # set by procedures (e.g. "catA_pre_tdp"), read by the SADPF
        self.sadpf = None
        if sadpf:
            from .sadpf import HeliSadpf
            self.sadpf = HeliSadpf(np.random.default_rng(seed + 7919))

    # ------------------------------------------------------------------------------------------
    def observe(self) -> np.ndarray:
        return self.x

    @property
    def nr_pct(self) -> float:
        return 100.0 * self.x[13] / self.p.omega100

    def n_running(self) -> int:
        return sum(1 for f in self.dyn.engine_failed if not f)

    def believed_running(self) -> list:
        """Engines the governor/FADEC treats as running: with the SADPF on, only what it has detected as failed
        is excluded (the control system does not know the injected fault); without it, the true state."""
        if self.sadpf is not None:
            return [i for i in range(2) if not self.sadpf.failed[i]]
        return [i for i in range(2) if not self.dyn.engine_failed[i]]

    def fail_engine(self, index: int):
        """Flame-out of engine `index` now (used by scenarios that fail an engine on a condition, e.g. at the TDP)."""
        if not self.dyn.engine_failed[index]:
            self.dyn.engine_failed[index] = True
            if self.fail_time is None:
                self.fail_time = self.t
            self.events.append({"t": self.t, "code": "engine_failure_injected", "engine": index + 1,
                                "message": f"Falha do motor {index + 1} (injetada)"})

    def set_cushion(self, on: bool = True):
        """Flare/landing in autorotation: collective now controls sink rate, spending rotor energy."""
        if on and not self.cushion:
            self.events.append({"t": self.t, "code": "cushion", "message": "Coletivo de amortecimento (pouso)"})
        self.cushion = on
        if on:
            self.autorotation = False

    def set_autorotation(self, on: bool = True):
        if on and not self.autorotation:
            self._nr_int = 0.0
            self._theta_auto = self.g.autorotation_collective
            self.events.append({"t": self.t, "code": "autorotation", "message": "Coletivo em autorrotação"})
        self.autorotation = on

    # ------------------------------------------------------------------------------------------
    def _sas(self, setpoint, dt: float) -> Controls:
        p, g, x = self.p, self.g, self.x
        pos_ref, vel_ref, yaw_ref = setpoint
        pos, vel, w = x[0:3], x[3:6], x[10:13]
        R = quat_to_rot(x[6:10])
        m = self.dyn.mass(x)
        grav = self.atm.gravity
        R_, n, v_rel_b, aero = self.dyn.aero(x, self.u)
        self._last_aero = aero
        rs = aero.rotor

        # horizontal: position -> velocity -> acceleration
        kp = g.pos_wn ** 2 / (2 * g.pos_zeta * g.pos_wn)
        v_cmd = np.asarray(vel_ref[:2], float) + kp * (np.asarray(pos_ref[:2], float) - pos[:2])
        sp = np.linalg.norm(v_cmd)
        if sp > g.max_speed:
            v_cmd *= g.max_speed / sp
        kv = 2 * g.pos_zeta * g.pos_wn
        e_v = v_cmd - vel[:2]
        self._vel_int = np.clip(self._vel_int + e_v * dt, -10.0, 10.0)
        a_xy = kv * e_v + g.vel_ki * self._vel_int
        # drag the thrust has to balance (world frame)
        v_air = vel - (self.dyn.wind if self.dyn.wind is not None else 0.0)
        drag_w = -0.5 * aero.rho * p.f_drag * np.linalg.norm(v_air[:2]) * np.append(v_air[:2], 0.0)
        # vertical
        vz_cmd = float(np.clip(vel_ref[2] + (pos_ref[2] - pos[2]) / g.vz_tau, -g.max_vz, g.max_vz))
        e_vz = vz_cmd - vel[2]
        self._vz_int = float(np.clip(self._vz_int + e_vz * dt, -5.0, 5.0))
        a_z = e_vz / g.vz_tau + g.vz_ki * self._vz_int
        f_des = m * np.array([a_xy[0], a_xy[1], grav + a_z]) - drag_w
        f_fin = fin_side_force(p, aero.rho, float(v_rel_b[0]), float(v_rel_b[1]))
        f_des -= R @ np.array([0.0, aero.tail_thrust + f_fin, 0.0])  # rotor tilt also balances tail + fin side force
        h_max = f_des[2] * math.tan(math.radians(g.max_tilt_deg))
        h = np.linalg.norm(f_des[:2])
        if h > h_max:
            f_des[:2] *= h_max / h

        # attitude (SO(3) error as in the drone controller)
        b3 = f_des / np.linalg.norm(f_des)
        b1c = np.array([math.cos(yaw_ref), math.sin(yaw_ref), 0.0])
        b2 = np.cross(b3, b1c)
        b2 /= np.linalg.norm(b2)
        b1 = np.cross(b2, b3)
        R_des = np.column_stack([b1, b2, b3])
        e_R = 0.5 * _vee(R_des.T @ R - R.T @ R_des)
        I = self.dyn.inertia
        M = -(I * g.att_wn ** 2) @ e_R - (I * 2 * g.att_zeta * g.att_wn) @ w
        k_tilt = p.hub_k + p.hub_h * max(rs.thrust, 0.2 * m * grav)
        lim = p.cyclic_limit
        cyc_lat = float(np.clip(M[0] / k_tilt, -lim, lim))
        cyc_lon = float(np.clip(M[1] / k_tilt, -lim, lim))
        # yaw: tail-rotor thrust balancing the drive torque plus the heading correction
        omega = max(x[13], 1.0)
        p_main_drive = p.eta_tr * (x[16] + x[17]) - aero.p_tail - aero.p_acc
        q_drive = max(p_main_drive, 0.0) / omega
        e_yaw = math.atan2(math.sin(euler_from_quat(x[6:10])[2] - yaw_ref),
                           math.cos(euler_from_quat(x[6:10])[2] - yaw_ref))
        mz_des = -I[2, 2] * (g.yaw_wn ** 2 * e_yaw + 2 * g.yaw_zeta * g.yaw_wn * w[2])
        t_tr = (-p.rotor_dir * q_drive - p.yaw_damping * w[2] - mz_des) / p.tr_arm - f_fin  # fin unloads it
        pedal = float(np.clip(t_tr / max(tail_rotor_max_thrust(p, aero.rho, omega), 1.0), -1.0, 1.0))

        # collective
        vt = omega * p.R
        if self.collective_hold is not None:
            # the pilot lowers the collective at a finite rate after touchdown (not a step): a step unloads the
            # rotor faster than the engines can spool down (tau_e) and overspeeds it
            prev = float(self.u.collective)
            step = COLLECTIVE_LOWER_RATE * dt
            collective = prev + float(np.clip(self.collective_hold - prev, -step, step))
        elif self.autorotation:
            e_nr = (x[13] - self.nr_ref * p.omega100) / p.omega100
            self._nr_int = float(np.clip(self._nr_int + e_nr * dt, -0.5, 0.5))
            collective = self._theta_auto + g.nr_kp * e_nr + g.nr_ki * self._nr_int
        if self.collective_hold is None and not self.autorotation:
            t_req = float(f_des @ (R @ n))
            ct = t_req / (aero.rho * p.area * vt * vt)
            mu = rs.mu
            collective = 3.0 * (2.0 * ct / (p.sigma * p.a) + rs.lam / 2.0) / (1.0 + 1.5 * mu * mu)
            if not self.cushion:
                # NR droop protection (power limit): give back collective when the rotor slows below 98.5 %
                droop = max(0.0, 0.985 - x[13] / p.omega100)
                collective -= 6.0 * droop
        collective = float(np.clip(collective, p.theta_min, p.theta_max))

        # governor
        p_need = (rs.power + aero.p_tail + aero.p_acc) / p.eta_tr
        p_need += p.I_rotor * omega * (p.omega100 - x[13]) / g.gov_tau / p.eta_tr
        running = self.believed_running()
        n_run = len(running)
        if self.sadpf is not None:
            t_fail = None if self.sadpf.detect_t is None else self.t - self.sadpf.detect_t
        else:
            t_fail = None if self.fail_time is None else self.t - self.fail_time
        self.rating = rating_for(max(n_run, 1), t_fail)
        lim_w = engine_limit_w(p, self.atm, pos[2], self.rating, max(n_run, 1))
        each = max(p_need, 0.0) / max(n_run, 1)
        # each engine control unit also governs NR: when the other engine falls clearly behind its share
        # (more than COMP_BAND of it, e.g. a flame-out not yet declared failed), this one picks up the missing
        # power, up to its current rating limit (the OEI ratings are armed only after the SADPF declares it)
        self.p_need_w = max(p_need, 0.0)
        cmd = []
        for i in range(2):
            if i not in running:
                cmd.append(0.0)
                continue
            others = [j for j in running if j != i]
            miss = sum(max(each - max(self.x[16 + j], 0.0) - COMP_BAND * each, 0.0) for j in others)
            cmd.append(min(each + miss, lim_w))
        p_cmd = tuple(cmd)
        return Controls(collective, cyc_lon, cyc_lat, pedal, p_cmd)

    def step(self, setpoint) -> None:
        for f in self.faults:
            if self.t >= f.t_start and not self.dyn.engine_failed[f.index]:
                self.dyn.engine_failed[f.index] = True
                if self.fail_time is None:
                    self.fail_time = self.t
                self.events.append({"t": self.t, "code": "engine_failure", "engine": f.index + 1,
                                    "message": f"Falha do motor {f.index + 1} (injetada)"})
        if self._k % self.ctrl_every == 0:
            cdt = self.ctrl_every * self.dt
            if self.wind is not None:
                self.dyn.wind = self.wind.step(cdt)
            if self.sadpf is not None:
                self.sadpf.update(self, cdt)
            self.u = self._sas(setpoint, cdt)
        self.x = self.dyn.rk4_step(self.x, self.u, self.dt)
        self.t += self.dt
        self._k += 1

    def record(self) -> None:
        x = self.x
        R_, n, v_rel_b, a = self.dyn.aero(x, self.u)
        roll, pitch, yaw = euler_from_quat(x[6:10])
        p_avail = engine_limit_w(self.p, self.atm, x[2], self.rating, self.n_running()) * self.n_running()
        self.telemetry.rows.append({
            "t": self.t, "x": x[0], "y": x[1], "z": x[2], "vx": x[3], "vy": x[4], "vz": x[5],
            "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch), "yaw_deg": math.degrees(yaw),
            "nr_pct": self.nr_pct, "ias_kt": a.airspeed * math.sqrt(a.rho / RHO0) / KT, "tas_ms": a.airspeed,
            "p_main_kw": a.rotor.power / 1e3, "p_tail_kw": a.p_tail / 1e3,
            "p_eng1_kw": x[16] / 1e3, "p_eng2_kw": x[17] / 1e3, "p_avail_kw": p_avail / 1e3,
            "collective_deg": math.degrees(self.u.collective), "pedal": self.u.pedal,
            "fin_n": fin_side_force(self.p, a.rho, float(v_rel_b[0]), float(v_rel_b[1])), "k_ge": a.rotor.k_ge, "vrs": float(a.rotor.vrs), "v_h": a.rotor.v_h,
            "mass_kg": self.dyn.mass(x), "fuel_kg": x[18], "on_ground": float(a.on_ground),
            "rating": {"TO": 0, "OEI30": 1, "OEI2": 2, "OEIC": 3}[self.rating], "autorotation": float(self.autorotation),
            "cushion": float(self.cushion), "sadpf_level": float(self.sadpf.level) if self.sadpf else 0.0,
            "agl": x[2] - self.dyn.terrain.height(x[0], x[1]),
            "n_eng": float(2 - sum(bool(f) for f in self.dyn.engine_failed)),  # engines physically running
        })


def scripted(fn: Callable[[float, np.ndarray], tuple], at: float, action: Callable[[], None]):
    """Wrap a guidance function to call `action` once at simulated time `at` (e.g. set autorotation)."""
    state = {"done": False}

    def g(t, x):
        if not state["done"] and t >= at:
            state["done"] = True
            action()
        return fn(t, x)
    return g
