"""
Actuator models: collective-pitch rotors and reaction-control thrusters.

Rotor model (hover / low-speed, momentum theory with figure of merit)
--------------------------------------------------------------------
The rotor spins at a constant tip speed (as Ingenuity and the NASA Mars Science
Helicopter concept do) and thrust is controlled with collective pitch.

    tip speed      V_t   = min(M_tip_max * a(h), V_t_motor_max)
    max thrust     T_max = (C_T/sigma)_max * sigma * rho * A * V_t^2
    shaft power    P     = T^(3/2) / (FM * sqrt(2 * rho * A))
    electric power P_e   = P / eta_motor
    reaction torque Q    = P / Omega,  Omega = V_t / R

Because rho appears in both T_max and P, a rotor produces zero thrust in vacuum
and needs roughly 1/sqrt(rho) more power for the same thrust in thin air - the
two effects the legacy engine ignored.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .atmosphere import G0

EARTH_SL_DENSITY = 1.225
EARTH_SL_SOUND = 340.3


@dataclass
class Rotor:
    radius: float  # m
    solidity: float  # blade area / disk area
    ct_sigma_max: float  # blade-loading limit (C_T / sigma) before stall
    figure_of_merit: float  # hover aerodynamic efficiency, 0..1
    motor_efficiency: float  # electrical -> shaft, 0..1
    max_tip_mach: float = 0.8
    max_tip_speed: float = 1e9  # m/s, motor/structural limit
    time_constant: float = 0.05  # s, first-order thrust response
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body frame, m
    spin: int = 1  # +1 = counter-clockwise seen from above (reaction torque is -z)
    health: float = 1.0  # 1 = nominal, 0 = failed (used by fault injection)
    # Fraction of the commanded thrust the rotor actually delivers (loss-of-effectiveness fault,
    # e.g. a chipped blade or a degraded motor). The motor still spends the power and reaction
    # torque of the commanded thrust, so the fault shows up as missing force, not missing power.
    effectiveness: float = 1.0

    @property
    def disk_area(self) -> float:
        return math.pi * self.radius ** 2

    def tip_speed(self, rho: float, sound_speed: float) -> float:
        if rho <= 0.0 or sound_speed <= 0.0:
            return 0.0
        return min(self.max_tip_mach * sound_speed, self.max_tip_speed)

    def max_thrust(self, rho: float, sound_speed: float) -> float:
        vt = self.tip_speed(rho, sound_speed)
        return self.health * self.ct_sigma_max * self.solidity * rho * self.disk_area * vt ** 2

    def shaft_power(self, thrust: float, rho: float) -> float:
        if thrust <= 0.0 or rho <= 0.0:
            return 0.0
        return thrust ** 1.5 / (self.figure_of_merit * math.sqrt(2.0 * rho * self.disk_area))

    def electrical_power(self, thrust: float, rho: float) -> float:
        return self.shaft_power(thrust, rho) / self.motor_efficiency

    def torque(self, thrust: float, rho: float, sound_speed: float) -> float:
        vt = self.tip_speed(rho, sound_speed)
        if vt <= 0.0:
            return 0.0
        omega = vt / self.radius
        return self.shaft_power(thrust, rho) / omega

    def rpm(self, rho: float, sound_speed: float) -> float:
        return self.tip_speed(rho, sound_speed) / self.radius * 60.0 / (2.0 * math.pi)

    @classmethod
    def from_legacy(cls, max_thrust_sea_level: float, diameter: float, efficiency: float,
                    position: np.ndarray, spin: int) -> "Rotor":
        """Rotor matching a legacy config's 'max thrust per motor' at Earth sea level.

        The legacy files give a fixed thrust number; here it is converted to a
        blade-loading coefficient so thrust scales with air density.
        """
        radius = max(diameter / 2.0, 0.01)
        solidity = 0.10
        tip_limit = 0.6 * EARTH_SL_SOUND  # typical small-UAV tip speed limit
        area = math.pi * radius ** 2
        ct_sigma = max_thrust_sea_level / (solidity * EARTH_SL_DENSITY * area * tip_limit ** 2)
        return cls(radius=radius, solidity=solidity, ct_sigma_max=ct_sigma,
                   figure_of_merit=0.6, motor_efficiency=max(min(efficiency, 0.95), 0.3),
                   max_tip_mach=0.8, max_tip_speed=tip_limit, position=position, spin=spin)


@dataclass
class Thruster:
    """Throttleable chemical thruster (monopropellant / cold gas)."""

    max_force: float  # N
    isp: float  # s (vacuum specific impulse)
    position: np.ndarray  # body frame, m
    direction: np.ndarray  # unit vector of the force ON the vehicle, body frame
    time_constant: float = 0.02
    health: float = 1.0

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)
        d = np.asarray(self.direction, dtype=float)
        self.direction = d / np.linalg.norm(d)

    def mass_flow(self, force: float) -> float:
        """Propellant mass flow (kg/s) for a given force: mdot = F / (Isp * g0)."""
        return max(force, 0.0) / (self.isp * G0)
