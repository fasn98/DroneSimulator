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


def load_body(name: str, config_path: str | Path = "config/environments.json") -> Body:
    """Build a Body from the legacy environments.json format."""
    path = Path(config_path)
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[2] / path
    with open(path) as f:
        cfg = json.load(f)[name]

    atm = cfg.get("atmosphere", {})
    vacuum = atm.get("type") == "vacuum" or atm.get("sea_level_pressure", 0.0) <= 0.0
    wind = cfg.get("wind", {})
    wind_on = bool(wind.get("enabled", False)) and not vacuum
    return Body(
        name=name,
        gravity=float(cfg["gravity"]),
        has_atmosphere=not vacuum,
        surface_pressure=float(atm.get("sea_level_pressure", 0.0)),
        surface_temperature=float(atm.get("sea_level_temperature", 250.0)),
        lapse_rate=float(atm.get("temperature_lapse_rate", 0.0)),
        gas_constant=float(atm.get("gas_constant", 0.0)),
        gamma=GAMMA_BY_BODY.get(name, 1.4),
        wind_mean=float(wind.get("base_speed", 0.0)) if wind_on else 0.0,
        # gust_factor in the legacy config is a peak multiplier; treat (factor-1)*mean/2 as 1-sigma
        wind_gust_std=(max(float(wind.get("gust_factor", 1.0)) - 1.0, 0.0)
                       * float(wind.get("base_speed", 0.0)) / 2.0) if wind_on else 0.0,
    )
