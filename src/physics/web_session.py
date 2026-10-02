"""
Web session: runs one mission of the Twin v2 physics core for the web app.

`WebSession` owns a TwinSimulator, the mission state (route follower, progress,
success / failure) and builds the telemetry dict the front-end consumes. It has
no Flask or SocketIO dependency, so it is unit tested directly
(tests/test_web_session.py); simulation_server.py only paces it in real time
and forwards the dicts.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from .atmosphere import Body, body_from_config, load_body
from .dynamics import euler_from_quat
from .simulator import TwinSimulator, waypoint_route
from .sizing import hover_report
from .vehicle import DEFAULT_MODELS, load_vehicle

BODY_NAMES_PT = {"earth": "na Terra", "mars": "em Marte", "moon": "na Lua"}

DEFAULT_CRUISE_CAP = 5.0  # m/s, same order as the legacy kinematic loop
DEFAULT_CLIMB_RATE = 1.5  # m/s
GROUND_TIMEOUT = 20.0  # s of simulated time without lifting off -> mission failed
LIFTOFF_MARGIN = 0.3  # m above the resting height that counts as "airborne"

LEGACY_KEYS = ("timestamp", "position", "velocity", "attitude", "altitude", "ground_speed", "vertical_speed",
               "vector_speed", "mission_progress", "current_waypoint", "mission_status", "total_distance")
PHYSICS_KEYS = ("power_w", "battery_wh", "battery_pct", "propellant_kg", "air_density", "tip_mach",
                "thrust_to_weight", "on_ground", "wind_speed")


def _num(v: Any, digits: int = 6) -> Optional[float]:
    """Plain, finite Python float (None for inf / NaN) so the dict is strict-JSON safe."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return round(f, digits)


def _decimal_pt(v: float, digits: int = 2) -> str:
    return f"{v:.{digits}f}".replace(".", ",")


class WebSession:
    """One mission flown by the physics core, advanced in chunks by the caller."""

    def __init__(self, vehicle_name: str, mission: Dict[str, Any], environment: str = "earth",
                 environment_config: Optional[Dict[str, Any]] = None, dt: float = 0.005, seed: int = 0,
                 models_path: str | Path = DEFAULT_MODELS, ground_timeout: float = GROUND_TIMEOUT):
        self.vehicle = load_vehicle(vehicle_name, models_path)
        with open(models_path) as f:
            self.envelope = json.load(f).get(vehicle_name, {}).get("flight_envelope", {}) or {}
        self.vehicle_name = vehicle_name
        self.environment = environment
        if environment_config is not None:
            self.body: Body = body_from_config(environment or "custom", environment_config, prefer_density=True)
        else:
            self.body = load_body(environment)
        self.sim = TwinSimulator(self.vehicle, self.body, dt=dt, seed=seed)
        self.dt = dt
        self.ground_timeout = ground_timeout

        # --- feasibility (closed form, before flying) ----------------------
        self.report = hover_report(self.vehicle, self.body)
        self.warnings: List[str] = self.vehicle.validate(self.body)
        self.can_fly = bool(self.report.can_hover)
        where = BODY_NAMES_PT.get(self.body.name, f"no ambiente '{self.body.name}'")
        tw = self.report.thrust_to_weight
        self.warning_pt: Optional[str] = None
        if not self.can_fly:
            self.warning_pt = f"Este veículo não consegue decolar {where} (T/W = {_decimal_pt(tw)})"
        elif tw < 1.3:
            self.warning_pt = f"Margem de controle baixa {where} (T/W = {_decimal_pt(tw)} < 1,3)"

        # --- mission -------------------------------------------------------
        self.mission = mission or {}
        wps = self.mission.get("waypoints", []) or []
        if not wps:
            wps = [{"x": 0.0, "y": 0.0, "z": 10.0, "tolerance": 2.0, "duration": 10.0}]
        self.waypoints = [[float(w.get("x", 0.0)), float(w.get("y", 0.0)), float(w.get("z", 0.0))] for w in wps]
        acceptance = [float(w.get("tolerance", 2.0)) for w in wps]
        hold = [float(w.get("duration", 5.0)) for w in wps]
        self.cruise_speed = self._cruise_speed()
        self.climb_rate = float(self.mission.get("climb_rate", min(DEFAULT_CLIMB_RATE, self.cruise_speed)))
        # Reference acceleration: half of what the tilt limit allows in this gravity, so the
        # position loop keeps margin to reject wind (Moon 0.38, Mars 0.86, Earth capped at 1.0 m/s^2).
        tilt = math.radians(self.sim.ctrl.g.max_tilt_deg)
        self.ref_accel = float(self.mission.get("max_accel", min(0.5 * self.body.gravity * math.tan(tilt), 1.0)))
        self.route = waypoint_route(self.waypoints, cruise_speed=self.cruise_speed, climb_rate=self.climb_rate,
                                    hold_time=hold, acceptance=acceptance, max_accel=self.ref_accel)
        criteria = self.mission.get("success_criteria", {}) or {}
        self.time_limit = float(criteria.get("flight_time_max", 900.0))

        self.status = "executing"  # executing | completed | failed | timeout
        self.failure_reason: Optional[str] = None
        self.total_distance = 0.0
        self.rest_height = float(self.sim.x[2])
        self.max_height = self.rest_height
        self.n = self.sim.dyn.n_act

    # ------------------------------------------------------------------ #
    def _cruise_speed(self) -> float:
        if "cruise_speed" in self.mission:
            return float(self.mission["cruise_speed"])
        candidates = [DEFAULT_CRUISE_CAP]
        mission_max = (self.mission.get("constraints", {}) or {}).get("max_speed")
        if mission_max:
            candidates.append(float(mission_max))
        if self.envelope.get("max_speed"):
            candidates.append(float(self.envelope["max_speed"]))
        return max(min(candidates), 0.5)

    @property
    def t(self) -> float:
        return self.sim.t

    @property
    def finished(self) -> bool:
        return self.status != "executing"

    @property
    def progress(self) -> float:
        """Fraction of waypoints completed (0..1)."""
        return float(self.route.progress())

    @property
    def current_waypoint(self) -> int:
        return int(self.route.index())

    # ------------------------------------------------------------------ #
    def advance(self, duration: float) -> None:
        """Integrate `duration` seconds of simulated time (stops early when the mission ends)."""
        steps = int(round(duration / self.dt))
        sim, n = self.sim, self.n
        for _ in range(steps):
            if self.finished:
                return
            prev = sim.x[0:3].copy()
            sim.step(self.route(sim.t, sim.x))
            x = sim.x
            self.total_distance += float(np.linalg.norm(x[0:3] - prev))
            if x[2] > self.max_height:
                self.max_height = float(x[2])
            self._check_end(x, n)

    def _check_end(self, x: np.ndarray, n: int) -> None:
        if self.route.progress() >= 1.0:
            self.status = "completed"
            return
        airborne_ever = self.max_height > self.rest_height + LIFTOFF_MARGIN
        if not airborne_ever and self.sim.t >= self.ground_timeout:
            self.status = "failed"
            self.failure_reason = self.warning_pt if not self.can_fly else (
                f"O veículo não decolou em {self.ground_timeout:.0f} s")
            return
        # Tip-over: body z axis more than 60 deg from vertical (R[2, 2] = 1 - 2(qx^2 + qy^2))
        if 1.0 - 2.0 * (x[7] * x[7] + x[8] * x[8]) < 0.5:
            self.status = "failed"
            self.failure_reason = "Veículo capotou (inclinação > 60°)"
            return
        if self.vehicle.rotors and self.vehicle.battery_wh > 0 and x[13 + n] <= 0.0:
            self.status = "failed"
            self.failure_reason = "Bateria esgotada"
            return
        if self.vehicle.thrusters and self.vehicle.propellant_mass > 0 and x[14 + n] <= 0.0:
            self.status = "failed"
            self.failure_reason = "Propelente esgotado"
            return
        if self.sim.t >= self.time_limit:
            self.status = "timeout"
            self.failure_reason = f"Tempo limite da missão ({self.time_limit:.0f} s) atingido"

    # ------------------------------------------------------------------ #
    def feasibility(self) -> Dict[str, Any]:
        r = self.report
        return {
            "vehicle": self.vehicle_name,
            "body": self.body.name,
            "can_fly": self.can_fly,
            "thrust_to_weight": _num(r.thrust_to_weight, 3),
            "gross_mass_kg": _num(r.gross_mass_kg, 3),
            "hover_power_w": _num(r.hover_power_w, 1),
            "hover_endurance_min": _num(r.endurance_min, 2),
            "tip_mach": _num(r.tip_mach, 3),
            "warning": self.warning_pt,
            "warning_level": None if self.warning_pt is None else ("warning" if self.can_fly else "danger"),
            "warnings": list(self.warnings),
        }

    def environment_data(self) -> Dict[str, Any]:
        """Same keys as the legacy environment panel, from the physics model."""
        alt = max(float(self.sim.x[2]), 0.0)
        return {
            "gravity": _num(self.body.gravity, 4),
            "air_density": _num(self.body.density(alt), 6),
            "temperature": _num(self.body.temperature(alt) - 273.15, 2),
            "wind_speed": _num(np.linalg.norm(self.sim.dyn.wind), 3),
        }

    def telemetry(self) -> Dict[str, Any]:
        sim, n = self.sim, self.n
        x = sim.x
        out = sim.dyn.outputs(x)
        roll, pitch, yaw = euler_from_quat(x[6:10])
        vel = x[3:6]
        alt = float(x[2])
        mass = sim.dyn.mass(x)
        weight = mass * self.body.gravity
        tw = self.vehicle.max_vertical_force(self.body, max(alt, 0.0)) / weight if weight > 0 else 0.0
        battery = float(x[13 + n])
        cap = self.vehicle.battery_wh
        return {
            # --- legacy fields (names and units unchanged) ---
            "timestamp": _num(sim.t, 3),
            "position": {"x": _num(x[0], 4), "y": _num(x[1], 4), "z": _num(x[2], 4)},
            "velocity": {"x": _num(vel[0], 4), "y": _num(vel[1], 4), "z": _num(vel[2], 4)},
            "attitude": {"roll": _num(roll), "pitch": _num(pitch), "yaw": _num(yaw)},  # rad
            "angular_velocity": {"x": _num(x[10]), "y": _num(x[11]), "z": _num(x[12])},  # rad/s, body
            "altitude": _num(alt, 4),
            "ground_speed": _num(np.linalg.norm(vel[:2]), 4),
            "vertical_speed": _num(vel[2], 4),
            "vector_speed": _num(np.linalg.norm(vel), 4),
            "mission_progress": _num(100.0 * self.progress, 2),  # percent, as before
            "current_waypoint": self.current_waypoint,
            "mission_status": self.status,
            "total_distance": _num(self.total_distance, 3),
            # --- Twin v2 physics ---
            "power_w": _num(out.electrical_power, 2),
            "battery_wh": _num(battery, 3),
            "battery_pct": _num(100.0 * battery / cap, 2) if cap > 0 else 0.0,
            "energy_consumed": _num(cap - battery, 3),  # Wh, read by database_service
            "propellant_kg": _num(x[14 + n], 4),
            "air_density": _num(out.density, 6),
            "tip_mach": _num(out.max_tip_mach, 4),
            "thrust_to_weight": _num(tw, 3),
            "on_ground": bool(out.on_ground),
            "wind_speed": _num(np.linalg.norm(sim.dyn.wind), 3),
        }
