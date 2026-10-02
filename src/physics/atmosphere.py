"""
Planetary atmosphere and gravity models for the Twin v2 physics core.

All quantities are SI. The troposphere-style model uses a constant lapse rate
and the body's own surface gravity in the hydrostatic exponent (the legacy
src/environment.py used Earth's 9.81 for every planet).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

G0 = 9.80665  # standard gravity, used only for specific impulse (Isp) conversion

# Ratio of specific heats per atmosphere type (CO2-dominated Mars vs. N2/O2 Earth)
GAMMA_BY_BODY = {"earth": 1.40, "mars": 1.29}


@dataclass
class Body:
    """A planetary body: gravity plus an optional atmosphere."""

    name: str
    gravity: float  # m/s^2 at the surface
    has_atmosphere: bool
    surface_pressure: float = 0.0  # Pa
    surface_temperature: float = 250.0  # K
    lapse_rate: float = 0.0  # K/m (negative = cools with height)
    gas_constant: float = 0.0  # J/(kg K)
    gamma: float = 1.4
    wind_mean: float = 0.0  # m/s
    wind_gust_std: float = 0.0  # m/s
    wind_direction_deg: float = 0.0

    # ------------------------------------------------------------------ #
    def temperature(self, altitude: float) -> float:
        if not self.has_atmosphere:
            return self.surface_temperature
        return max(self.surface_temperature + self.lapse_rate * altitude, 50.0)

    def pressure(self, altitude: float) -> float:
        if not self.has_atmosphere:
            return 0.0
        t0, L, R, g = self.surface_temperature, self.lapse_rate, self.gas_constant, self.gravity
        if abs(L) < 1e-9:
            return self.surface_pressure * math.exp(-g * altitude / (R * t0))
        return self.surface_pressure * (self.temperature(altitude) / t0) ** (-g / (R * L))

    def density(self, altitude: float) -> float:
        if not self.has_atmosphere:
            return 0.0
        return self.pressure(altitude) / (self.gas_constant * self.temperature(altitude))

    def speed_of_sound(self, altitude: float) -> float:
        if not self.has_atmosphere:
            return 0.0
        return math.sqrt(self.gamma * self.gas_constant * self.temperature(altitude))

    # ------------------------------------------------------------------ #
    @classmethod
    def with_surface_density(cls, base: "Body", density: float, temperature: float) -> "Body":
        """Copy of `base` whose surface state matches a given density and temperature.

        Useful to reproduce published design points (e.g. NASA MSH: 0.015 kg/m^3, -50 C).
        """
        b = cls(**{k: getattr(base, k) for k in base.__dataclass_fields__})
        b.surface_temperature = temperature
        b.surface_pressure = density * b.gas_constant * temperature
        return b


def body_from_config(name: str, cfg: dict, prefer_density: bool = False) -> Body:
    """Build a Body from an environments.json-style dict.

    With `prefer_density=True` (used for user-defined environments, whose UI asks
    for density rather than pressure) the surface pressure is derived from
    `sea_level_density` via p = rho R T, so the density the user typed is the
    density the rotors see.
    """
    atm = cfg.get("atmosphere", {}) or {}
    rho_in = float(atm.get("sea_level_density", 0.0) or 0.0)
    pressure = float(atm.get("sea_level_pressure", 0.0) or 0.0)
    use_density = prefer_density and "sea_level_density" in atm
    vacuum = atm.get("type") == "vacuum" or (rho_in <= 0.0 if use_density else pressure <= 0.0)
    gas_r = float(atm.get("gas_constant", 0.0) or 0.0)
    temp = float(atm.get("sea_level_temperature", 250.0) or 250.0)
    if not vacuum and gas_r <= 0.0:
        gas_r = 287.0
    if not vacuum and use_density:
        pressure = rho_in * gas_r * temp
    wind = cfg.get("wind", {}) or {}
    wind_on = bool(wind.get("enabled", False)) and not vacuum
    base = float(wind.get("base_speed", 0.0) or 0.0)
    return Body(
        name=name,
        gravity=float(cfg["gravity"]),
        has_atmosphere=not vacuum,
        surface_pressure=pressure if not vacuum else 0.0,
        surface_temperature=temp,
        lapse_rate=float(atm.get("temperature_lapse_rate", 0.0) or 0.0),
        gas_constant=gas_r if not vacuum else 0.0,
        gamma=GAMMA_BY_BODY.get(name, 1.4),
        wind_mean=base if wind_on else 0.0,
        # gust_factor in the legacy config is a peak multiplier; treat (factor-1)*mean/2 as 1-sigma
        wind_gust_std=(max(float(wind.get("gust_factor", 1.0)), 1.0) - 1.0) * base / 2.0 if wind_on else 0.0,
        wind_direction_deg=float(wind.get("direction", 0.0) or 0.0),
    )


def load_body(name: str, config_path: str | Path = "config/environments.json") -> Body:
    """Build a Body from the legacy environments.json format."""
    path = Path(config_path)
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[2] / path
    with open(path) as f:
        cfg = json.load(f)[name]
    return body_from_config(name, cfg)
