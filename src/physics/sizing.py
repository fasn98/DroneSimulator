"""
Steady-state sizing and feasibility analysis (no time integration).

These are the closed-form numbers a reviewer can check by hand; the time-domain
simulator must agree with them in hover.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Dict

from .atmosphere import Body
from .vehicle import Vehicle


@dataclass
class HoverReport:
    vehicle: str
    body: str
    gross_mass_kg: float
    weight_n: float
    max_vertical_force_n: float
    thrust_to_weight: float
    can_hover: bool
    density_kg_m3: float
    tip_mach: float
    rotor_rpm: float
    ct_sigma_hover: float
    hover_power_w: float  # electrical, incl. avionics + heaters
    endurance_min: float  # hover endurance on the full battery / tank
    propellant_flow_kg_s: float

    def as_dict(self) -> Dict:
        return asdict(self)


def hover_report(vehicle: Vehicle, body: Body, altitude: float = 0.0) -> HoverReport:
    rho, a = body.density(altitude), body.speed_of_sound(altitude)
    weight = vehicle.gross_mass * body.gravity
    fmax = vehicle.max_vertical_force(body, altitude)
    can = bool(fmax >= weight)
    power = flow = 0.0
    tip_mach = rpm = ct_sig = 0.0
    endurance = 0.0
    if vehicle.rotors:
        r0 = vehicle.rotors[0]
        n = len(vehicle.rotors)
        t_each = weight / n
        vt = r0.tip_speed(rho, a)
        tip_mach = vt / a if a > 0 else 0.0
        rpm = r0.rpm(rho, a)
        if rho > 0 and vt > 0:
            ct_sig = t_each / (r0.solidity * rho * r0.disk_area * vt ** 2)
        if can:
            power = n * r0.electrical_power(t_each, rho) + vehicle.avionics_w + vehicle.heater_w
            endurance = vehicle.battery_wh / power * 60.0 if power > 0 else 0.0
        else:
            power = math.inf
    if vehicle.thrusters:
        main = [t for t in vehicle.thrusters if t.direction[2] > 0.5]
        isp = main[0].isp if main else vehicle.thrusters[0].isp
        flow = weight / (isp * 9.80665)
        if can and flow > 0:
            # Hover time with mass decreasing as propellant burns: t = Isp g0 / g * ln(m0 / m1)
            m0 = vehicle.gross_mass
            m1 = m0 - vehicle.propellant_mass
            endurance = isp * 9.80665 / body.gravity * math.log(m0 / m1) / 60.0
    return HoverReport(vehicle.name, body.name, vehicle.gross_mass, weight, fmax, fmax / weight, can, rho,
                       tip_mach, rpm, ct_sig, power, endurance, flow)


def max_hover_mass(vehicle: Vehicle, body: Body, altitude: float = 0.0) -> float:
    """Largest gross mass the actuators can hold at T/W = 1."""
    return vehicle.max_vertical_force(body, altitude) / body.gravity


def ideal_hover_power(mass: float, body: Body, total_disk_area: float, altitude: float = 0.0) -> float:
    """Momentum-theory ideal power (W): W^1.5 / sqrt(2 rho A)."""
    rho = body.density(altitude)
    w = mass * body.gravity
    return w ** 1.5 / math.sqrt(2 * rho * total_disk_area)


def lunar_delta_v(isp: float, wet_mass: float, dry_mass: float) -> float:
    """Tsiolkovsky delta-v budget (m/s)."""
    return isp * 9.80665 * math.log(wet_mass / dry_mass)
