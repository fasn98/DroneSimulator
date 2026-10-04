"""
ISA atmosphere with a temperature deviation and a heliport elevation.

The usual aviation convention: a height h above mean sea level has the ISA pressure p_ISA(h)
(pressure altitude) and the temperature T_ISA(h) + dT. Density follows from p = rho R T, and the
density altitude is the ISA height with that density. The simulator works in heights above the
heliport (AGL); `at(z_agl)` adds the heliport elevation.

ISA constants: ICAO Doc 7488 / ISO 2533 (T0 288.15 K, p0 101325 Pa, lapse 6.5 K/km, R 287.053, g0 9.80665).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..atmosphere import Body

T0, P0, LAPSE, R_AIR, G0, GAMMA = 288.15, 101325.0, 0.0065, 287.053, 9.80665, 1.4
RHO0 = P0 / (R_AIR * T0)


def isa_pressure(h: float) -> float:
    return P0 * (1.0 - LAPSE * h / T0) ** (G0 / (R_AIR * LAPSE))


def isa_temperature(h: float) -> float:
    return T0 - LAPSE * h


def isa_density(h: float) -> float:
    return isa_pressure(h) / (R_AIR * isa_temperature(h))


def density_altitude(rho: float) -> float:
    """ISA height with density rho (inverse of isa_density)."""
    return T0 / LAPSE * (1.0 - (rho / RHO0) ** (1.0 / (G0 / (R_AIR * LAPSE) - 1.0)))


@dataclass
class HeliAtmosphere:
    elevation_m: float = 0.0  # heliport elevation above MSL
    delta_t: float = 0.0  # ISA deviation, K
    wind_mean: float = 0.0
    wind_gust_std: float = 0.0
    wind_direction_deg: float = 0.0

    def temperature(self, z_agl: float) -> float:
        return isa_temperature(self.elevation_m + z_agl) + self.delta_t

    def pressure(self, z_agl: float) -> float:
        return isa_pressure(self.elevation_m + z_agl)

    def density(self, z_agl: float) -> float:
        return self.pressure(z_agl) / (R_AIR * self.temperature(z_agl))

    def speed_of_sound(self, z_agl: float) -> float:
        return math.sqrt(GAMMA * R_AIR * self.temperature(z_agl))

    def density_altitude(self, z_agl: float) -> float:
        return density_altitude(self.density(z_agl))

    @property
    def gravity(self) -> float:
        return G0

    def as_body(self) -> Body:
        """A `Body` for the shared WindModel (only the wind fields are used)."""
        return Body(name="earth", gravity=G0, has_atmosphere=True, surface_pressure=self.pressure(0.0),
                    surface_temperature=self.temperature(0.0), lapse_rate=-LAPSE, gas_constant=R_AIR,
                    wind_mean=self.wind_mean, wind_gust_std=self.wind_gust_std,
                    wind_direction_deg=self.wind_direction_deg)
