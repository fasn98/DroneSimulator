"""
Vehicle definitions for the Twin v2 physics core.

A vehicle is loaded from config/drone_models.json. Entries with a "physics"
block are used as-is; older entries are converted from their legacy fields so
the whole catalogue runs on the same physics (and gets checked by `validate`).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np

from .actuators import Rotor, Thruster
from .atmosphere import Body

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS = REPO_ROOT / "config" / "drone_models.json"


@dataclass
class Vehicle:
    name: str
    kind: str  # "multirotor" | "hopper"
    mass_dry: float  # kg, everything except payload and propellant (battery included)
    inertia: np.ndarray  # 3x3, kg m^2, about the centre of mass, body frame
    drag_area: float  # Cd * A, m^2
    payload: float = 0.0
    propellant_mass: float = 0.0
    battery_wh: float = 0.0
    avionics_w: float = 0.0
    heater_w: float = 0.0
    rotors: List[Rotor] = field(default_factory=list)
    thrusters: List[Thruster] = field(default_factory=list)
    gear_radius: float = 0.5  # m, footprint of the landing legs
    gear_height: float = 0.3  # m, distance from CG down to the feet
    description: str = ""
    notes: str = ""

    @property
    def gross_mass(self) -> float:
        return self.mass_dry + self.payload + self.propellant_mass

    @property
    def actuator_count(self) -> int:
        return len(self.rotors) + len(self.thrusters)

    # ------------------------------------------------------------------ #
    def max_vertical_force(self, body: Body, altitude: float = 0.0) -> float:
        rho, a = body.density(altitude), body.speed_of_sound(altitude)
        f = sum(r.max_thrust(rho, a) for r in self.rotors)
        f += sum(t.health * t.max_force * max(t.direction[2], 0.0) for t in self.thrusters)
        return f

    def thrust_to_weight(self, body: Body, altitude: float = 0.0) -> float:
        return self.max_vertical_force(body, altitude) / (self.gross_mass * body.gravity)

    def validate(self, body: Body) -> List[str]:
        """Physical sanity checks. Returns human-readable warnings (empty = OK)."""
        warnings = []
        tw = self.thrust_to_weight(body)
        if tw < 1.0:
            warnings.append(f"{self.name}: cannot lift off on {body.name} (T/W = {tw:.2f} < 1)")
        elif tw < 1.3:
            warnings.append(f"{self.name}: low control margin on {body.name} (T/W = {tw:.2f} < 1.3)")
        for i, r in enumerate(self.rotors):
            if r.ct_sigma_max > 0.20:
                warnings.append(f"{self.name}: rotor {i} blade loading C_T/sigma = {r.ct_sigma_max:.2f} "
                                f"exceeds ~0.20 (stall); the configured thrust is not achievable")
                break
        n = len(self.rotors)
        if n >= 2:
            pts = np.array([r.position[:2] for r in self.rotors])
            d = min(np.linalg.norm(pts[i] - pts[j]) for i in range(n) for j in range(i + 1, n))
            if d < 2 * self.rotors[0].radius * 0.98:
                warnings.append(f"{self.name}: rotor disks overlap (spacing {d:.2f} m < "
                                f"diameter {2 * self.rotors[0].radius:.2f} m)")
        return warnings


# ---------------------------------------------------------------------- #
# Loading
# ---------------------------------------------------------------------- #
def _ring(count: int, radius: float, offset_deg: float, z: float) -> List[np.ndarray]:
    return [np.array([radius * math.cos(math.radians(offset_deg + 360.0 * i / count)),
                      radius * math.sin(math.radians(offset_deg + 360.0 * i / count)), z])
            for i in range(count)]


def _from_physics_block(name: str, cfg: dict) -> Vehicle:
    p = cfg["physics"]
    kind = p.get("kind", "multirotor")
    inertia = np.diag(p["inertia"])
    gear = p.get("landing_gear", {})
    v = Vehicle(
        name=name, kind=kind, mass_dry=p["mass_dry"], inertia=inertia,
        drag_area=p.get("drag_area", 0.5), payload=p.get("payload", 0.0),
        propellant_mass=p.get("propellant_mass", 0.0), battery_wh=p.get("battery_wh", 0.0),
        avionics_w=p.get("avionics_w", 0.0), heater_w=p.get("heater_w", 0.0),
        gear_radius=gear.get("radius", 0.5), gear_height=gear.get("height", 0.3),
        description=cfg.get("description", ""), notes=p.get("notes", ""),
    )
    if "rotor" in p:
        lay, rc = p["rotor_layout"], p["rotor"]
        for i, pos in enumerate(_ring(lay["count"], lay["arm_length"], lay.get("angle_offset_deg", 0.0),
                                      lay.get("height", 0.0))):
            v.rotors.append(Rotor(
                radius=rc["radius"], solidity=rc["solidity"], ct_sigma_max=rc["ct_sigma_max"],
                figure_of_merit=rc["figure_of_merit"], motor_efficiency=rc["motor_efficiency"],
                max_tip_mach=rc.get("max_tip_mach", 0.8), max_tip_speed=rc.get("max_tip_speed", 1e9),
                time_constant=rc.get("time_constant", 0.05), position=pos,
                spin=1 if i % 2 == 0 else -1))
    if "thrusters" in p:
        th = p["thrusters"]
        main = th["main"]
        for pos in _ring(main["count"], main["radius"], main.get("angle_offset_deg", 0.0), main.get("z", 0.0)):
            v.thrusters.append(Thruster(main["max_force"], main["isp"], pos, [0, 0, 1]))
        rcs = th.get("rcs")
        if rcs:
            for pos in _ring(rcs["count"], rcs["radius"], rcs.get("angle_offset_deg", 0.0), rcs.get("z", 0.0)):
                tangent = np.array([-pos[1], pos[0], 0.0])
                for sign in (1.0, -1.0):  # one thruster each way -> pure yaw couples
                    v.thrusters.append(Thruster(rcs["max_force"], rcs["isp"], pos, sign * tangent))
    return v


def _from_legacy(name: str, cfg: dict) -> Vehicle:
    mass, dims, prop, aero = cfg["mass"], cfg["dimensions"], cfg["propulsion"], cfg.get("aerodynamics", {})
    n = int(prop.get("motor_count", 4))
    arm = dims.get("wingspan", 0.5) / 2.0
    gross = mass.get("mtow", mass.get("empty_weight", 1.0))
    r_eff = max(arm, 0.1)
    ixx = gross * (3 * r_eff ** 2 + dims.get("height", 0.2) ** 2) / 12.0
    v = Vehicle(
        name=name, kind="multirotor", mass_dry=gross,
        inertia=np.diag([ixx, ixx, gross * r_eff ** 2 / 2.0]),
        drag_area=aero.get("drag_coefficient", 0.5) * aero.get("reference_area", 0.1),
        battery_wh=prop.get("battery_capacity", 0.0), description=cfg.get("description", ""),
        gear_radius=max(arm * 0.5, 0.1), gear_height=max(dims.get("height", 0.2) / 2.0, 0.05),
        notes="Converted from legacy fields: gross mass = MTOW, thrust scaled with air density, "
              "battery_capacity interpreted as Wh.",
    )
    if prop.get("type") == "chemical_thrusters":
        v.kind = "hopper"
        v.propellant_mass = mass.get("fuel_capacity", 0.0)
        v.mass_dry = mass.get("empty_weight", gross)
        for pos in _ring(n, max(arm, 0.3), 45.0, 0.0):
            v.thrusters.append(Thruster(prop["max_thrust_per_motor"], prop.get("specific_impulse", 220.0),
                                        pos, [0, 0, 1]))
        return v
    offset = 45.0 if n == 4 else 0.0
    for i, pos in enumerate(_ring(n, arm, offset, 0.0)):
        v.rotors.append(Rotor.from_legacy(prop.get("max_thrust_per_motor", 10.0), dims.get("rotor_diameter", 0.3),
                                          prop.get("efficiency", 0.8), pos, 1 if i % 2 == 0 else -1))
    return v


def load_vehicle(name: str, config_path: str | Path = DEFAULT_MODELS, payload: Optional[float] = None) -> Vehicle:
    with open(config_path) as f:
        cfg = json.load(f)[name]
    v = _from_physics_block(name, cfg) if "physics" in cfg else _from_legacy(name, cfg)
    if payload is not None:
        v.payload = payload
    return v


def list_vehicles(config_path: str | Path = DEFAULT_MODELS) -> List[str]:
    with open(config_path) as f:
        return list(json.load(f).keys())
