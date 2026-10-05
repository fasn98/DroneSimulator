"""
Twin simulation loop: physics at 200 Hz, control at 50 Hz, stochastic wind.

Phase 3 options (all off by default, so phase 1/2 behaviour is unchanged):
    sensors=SensorConfig()   the controller flies on the EKF estimate built from
                             simulated sensors instead of the true state
    sadpf=True               fault detection, isolation and autonomous actions
    faults=[...]             RotorFault / SensorFault injected at their t_start
    plant_vehicle=...        the simulated vehicle differs from the model the
                             controller and the SADPF believe in (model mismatch)
"""

from __future__ import annotations

import copy
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

from .atmosphere import Body
from .control import GeometricController, Gains
from .dynamics import RigidBodyDynamics, euler_from_quat
from .ekf import NavigationEKF
from .sadpf import RotorFault, Sadpf, SadpfConfig
from .sensors import SensorConfig, SensorFault, SensorSuite
from .simloop import SimulationLoop, Telemetry, WindModel  # noqa: F401 (re-exported)
from .vehicle import Vehicle

Setpoint = Tuple[np.ndarray, np.ndarray, float]  # position, velocity, yaw


class TwinSimulator(SimulationLoop):
    def __init__(self, vehicle: Vehicle, body: Body, dt: float = 0.005, control_rate_hz: float = 50.0,
                 seed: int = 0, gains: Optional[Gains] = None, wind: bool = True,
                 sensors: Optional[SensorConfig] = None, sadpf: bool | SadpfConfig = False,
                 faults: Sequence[RotorFault | SensorFault] = (), plant_vehicle: Optional[Vehicle] = None):
        self.vehicle, self.body, self.dt = vehicle, body, dt
        # The plant is always a private copy, so injected faults never touch the caller's model.
        self.plant = copy.deepcopy(plant_vehicle if plant_vehicle is not None else vehicle)
        self.dyn = RigidBodyDynamics(self.plant, body)
        self.model_dyn = self.dyn if plant_vehicle is None else RigidBodyDynamics(vehicle, body)
        self.ctrl = GeometricController(self.model_dyn, gains)
        self.ctrl_every = max(int(round(1.0 / (control_rate_hz * dt))), 1)
        self.rng = np.random.default_rng(seed)
        self.wind = WindModel(body, self.rng) if wind else None
        self.x = self.dyn.initial_state()
        self.t = 0.0
        self.cmd = np.zeros(self.dyn.n_act)
        self.telemetry = Telemetry()
        self._k = 0
        self.rotor_faults = [f for f in faults if isinstance(f, RotorFault)]
        self.sensors = self.ekf = self.sadpf = None
        if sensors is not None or sadpf:
            cfg = sensors or SensorConfig()
            self.sensors = SensorSuite(body, np.random.default_rng(seed + 7919), cfg)
            self.sensors.faults = [f for f in faults if isinstance(f, SensorFault)]
            self.ekf = NavigationEKF(body.gravity, self.x, cfg, self.sensors.baro_altitude_noise(0.0))
        if sadpf:
            self.sadpf = Sadpf(vehicle, body, self.ctrl_every * dt, sadpf if isinstance(sadpf, SadpfConfig) else None)
        self._land_from: Optional[np.ndarray] = None

    def nav_state(self) -> np.ndarray:
        """The state the vehicle believes it is in (EKF estimate when sensors are simulated)."""
        return self.ekf.state_vector(self.x) if self.ekf is not None else self.x

    def observe(self) -> np.ndarray:
        return self.nav_state()

    def _landing_setpoint(self, setpoint: Setpoint) -> Setpoint:
        if self.sadpf is None or self.sadpf.land_requested is None:
            return setpoint
        if self._land_from is None:
            self._land_from = self.ekf.p.copy()
        p0 = self._land_from
        rate = self.sadpf.cfg.land_rate
        z = max(p0[2] - rate * (self.t - self.sadpf.land_requested), -0.3)
        vz = -rate if z > -0.3 else 0.0
        return np.array([p0[0], p0[1], z]), np.array([0.0, 0.0, vz]), setpoint[2]

    def step(self, setpoint: Setpoint):
        for f in self.rotor_faults:
            if self.t >= f.t_start and self.plant.rotors[f.index].effectiveness != f.effectiveness:
                self.plant.rotors[f.index].effectiveness = f.effectiveness
        if self._k % self.ctrl_every == 0:
            cdt = self.ctrl_every * self.dt
            if self.wind is not None:
                self.dyn.wind = self.wind.step(cdt)
            x_nav = self.x
            if self.ekf is not None:
                meas = self.sensors.sample(self.t, cdt, self.x, self.dyn.specific_force_body(self.x, self.cmd))
                self.ekf.step(meas, cdt)
                x_nav = self.ekf.state_vector(self.x)
                if self.sadpf is not None:
                    self.sadpf.update(self.t, self.cmd, self.ekf, meas, self.model_dyn.mass(x_nav),
                                      self.ctrl.alloc)
                    setpoint = self._landing_setpoint(setpoint)
            if self.sadpf is not None and self.sadpf.landed:
                self.cmd = np.zeros(self.dyn.n_act)  # safe state on the ground: motors off
            else:
                self.cmd = self.ctrl.update(x_nav, *setpoint, dt=cdt)
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
        if self.ekf is not None:
            row = self.telemetry.rows[-1]
            err = self.ekf.p - x[0:3]
            row.update({"est_x": self.ekf.p[0], "est_y": self.ekf.p[1], "est_z": self.ekf.p[2],
                        "pos_err_m": float(np.linalg.norm(err)), "pos_err_z": float(err[2]),
                        "sigma_pos_m": float(np.linalg.norm(self.ekf.position_sigma))})
        if self.sadpf is not None:
            row = self.telemetry.rows[-1]
            row.update({"sadpf_level": float(self.sadpf.level),
                        "residual_norm": float(np.linalg.norm(self.sadpf.fdi.residual)),
                        "rotor_eff_min_est": float(np.min(self.ctrl.alloc.effectiveness))
                        if len(self.ctrl.alloc.effectiveness) else 1.0})

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
