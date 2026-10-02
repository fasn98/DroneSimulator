"""
Twin v2 simulation loop: physics at 200 Hz, control at 50 Hz, stochastic wind.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .atmosphere import Body
from .control import GeometricController, Gains
from .dynamics import RigidBodyDynamics, euler_from_quat
from .vehicle import Vehicle

Setpoint = Tuple[np.ndarray, np.ndarray, float]  # position, velocity, yaw


class WindModel:
    """Mean wind plus first-order Gauss-Markov gusts (a simplified Dryden model)."""

    def __init__(self, body: Body, rng: np.random.Generator, correlation_time: float = 3.0):
        self.mean = body.wind_mean * np.array([np.cos(np.radians(body.wind_direction_deg)),
                                               np.sin(np.radians(body.wind_direction_deg)), 0.0])
        self.sigma = body.wind_gust_std
        self.tau = correlation_time
        self.rng = rng
        self.gust = np.zeros(3)

    def step(self, dt: float) -> np.ndarray:
        if self.sigma > 0:
            noise = self.rng.standard_normal(3) * np.array([1.0, 1.0, 0.3])
            self.gust += -self.gust * dt / self.tau + self.sigma * np.sqrt(2 * dt / self.tau) * noise
        return self.mean + self.gust


@dataclass
class Telemetry:
    rows: List[Dict[str, float]] = field(default_factory=list)

    def column(self, key: str) -> np.ndarray:
        return np.array([r[key] for r in self.rows])


class TwinSimulator:
    def __init__(self, vehicle: Vehicle, body: Body, dt: float = 0.005, control_rate_hz: float = 50.0,
                 seed: int = 0, gains: Optional[Gains] = None, wind: bool = True):
        self.vehicle, self.body, self.dt = vehicle, body, dt
        self.dyn = RigidBodyDynamics(vehicle, body)
        self.ctrl = GeometricController(self.dyn, gains)
        self.ctrl_every = max(int(round(1.0 / (control_rate_hz * dt))), 1)
        self.rng = np.random.default_rng(seed)
        self.wind = WindModel(body, self.rng) if wind else None
        self.x = self.dyn.initial_state()
        self.t = 0.0
        self.cmd = np.zeros(self.dyn.n_act)
        self.telemetry = Telemetry()
        self._k = 0

    def step(self, setpoint: Setpoint):
        if self._k % self.ctrl_every == 0:
            cdt = self.ctrl_every * self.dt
            if self.wind is not None:
                self.dyn.wind = self.wind.step(cdt)
            self.cmd = self.ctrl.update(self.x, *setpoint, dt=cdt)
        self.x = self.dyn.rk4_step(self.x, self.cmd, self.dt)
        self.t += self.dt
        self._k += 1

    def record(self):
        x, out, n = self.x, self.dyn.outputs(self.x), self.dyn.n_act
        roll, pitch, yaw = euler_from_quat(x[6:10])
        self.telemetry.rows.append({
            "t": self.t, "x": x[0], "y": x[1], "z": x[2], "vx": x[3], "vy": x[4], "vz": x[5],
            "roll_deg": np.degrees(roll), "pitch_deg": np.degrees(pitch), "yaw_deg": np.degrees(yaw),
            "power_w": out.electrical_power, "battery_wh": x[13 + n], "propellant_kg": x[14 + n],
            "density": out.density, "tip_mach": out.max_tip_mach, "on_ground": float(out.on_ground),
            "wind_x": self.dyn.wind[0], "wind_y": self.dyn.wind[1],
            "thrust_total_n": float(np.sum(np.clip(x[13:13 + n], 0, None))),
        })

    def run(self, duration: float, setpoint_fn: Callable[[float, np.ndarray], Setpoint],
            record_every: float = 0.1) -> Telemetry:
        rec_k = max(int(round(record_every / self.dt)), 1)
        steps = int(round(duration / self.dt))
        for i in range(steps):
            self.step(setpoint_fn(self.t, self.x))
            if i % rec_k == 0:
                self.record()
        self.record()
        return self.telemetry


# ---------------------------------------------------------------------- #
# Reference generators
# ---------------------------------------------------------------------- #
def hover_at(position: Sequence[float], yaw: float = 0.0, climb_rate: float = 1.0
             ) -> Callable[[float, np.ndarray], Setpoint]:
    """Climb from the ground at `climb_rate` to `position`, then hold it."""
    target = np.asarray(position, dtype=float)

    def fn(t: float, x: np.ndarray) -> Setpoint:
        z = min(target[2], climb_rate * t)
        vz = climb_rate if z < target[2] else 0.0
        return np.array([target[0], target[1], z]), np.array([0.0, 0.0, vz]), yaw
    return fn


def waypoint_route(waypoints: Sequence[Sequence[float]], cruise_speed: float = 3.0, climb_rate: float = 1.0,
                   hold_time: float | Sequence[float] = 3.0, acceptance: float | Sequence[float] = 1.5,
                   max_accel: Optional[float] = None) -> Callable[[float, np.ndarray], Setpoint]:
    """Carrot-chasing route follower over (x, y, z) waypoints, starting with a vertical climb.

    `hold_time` and `acceptance` may be scalars or one value per waypoint. With
    `max_accel` (m/s^2) the reference follows a trapezoidal speed profile: it
    speeds up at most at `max_accel` and slows down so that it stops on each
    waypoint. Without it the reference jumps straight to cruise speed (the
    Phase 1 behaviour), which a tilt-limited vehicle in low gravity cannot track
    without overshooting.
    """
    wps = [np.asarray(w, dtype=float) for w in waypoints]
    holds = [float(h) for h in hold_time] if np.ndim(hold_time) else [float(hold_time)] * len(wps)
    accs = [float(a) for a in acceptance] if np.ndim(acceptance) else [float(acceptance)] * len(wps)
    if len(holds) != len(wps) or len(accs) != len(wps):
        raise ValueError("hold_time / acceptance must have one value per waypoint")
    state = {"i": 0, "ref": None, "arrived": None, "t": 0.0, "v": 0.0}

    def fn(t: float, x: np.ndarray) -> Setpoint:
        if state["ref"] is None:
            state["ref"] = np.array([x[0], x[1], max(x[2], 0.0)])
            state["t"] = t
        dt = max(t - state["t"], 0.0)
        state["t"] = t
        i = min(state["i"], len(wps) - 1)
        target = wps[i]
        ref = state["ref"]
        d = target - ref
        dist = np.linalg.norm(d)
        speed = climb_rate if abs(d[2]) > np.linalg.norm(d[:2]) else cruise_speed
        if max_accel is not None:
            speed = min(speed, state["v"] + max_accel * dt, np.sqrt(2.0 * max_accel * dist))
        vel = np.zeros(3)
        if dist > speed * dt and dist > 1e-6:
            vel = d / dist * speed
            ref = ref + vel * dt
            state["v"] = speed
        else:
            ref = target.copy()
            state["v"] = 0.0
        state["ref"] = ref
        if state["i"] < len(wps) and np.linalg.norm(x[0:3] - target) < accs[i]:
            if state["arrived"] is None:
                state["arrived"] = t
            elif t - state["arrived"] >= holds[i]:
                state["i"] += 1
                state["arrived"] = None
        return ref, vel, 0.0
    fn.progress = lambda: state["i"] / len(wps)  # fraction of waypoints completed
    fn.index = lambda: state["i"]  # index of the active waypoint (== len(waypoints) when done)
    return fn
