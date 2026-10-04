"""
Vehicle templates: one simulation engine, several kinds of vehicle.

A `VehicleTemplate` bundles what differs between vehicles (plant model,
controller, default environment, telemetry fields, HUD label) behind one
interface, while the clock, run loop, wind and telemetry container come from
`simloop.SimulationLoop`. Selecting a template never changes another one:
the Mars drone template wraps `TwinSimulator` exactly as before (checked by
tests/test_template_regression.py against trajectories recorded before the
abstraction existed).

    from src.physics.templates import get_template
    sim = get_template("drone_marte").build(seed=1)
    tel = sim.run(30.0, get_template("drone_marte").default_guidance())
"""

from __future__ import annotations

import dataclasses
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Tuple

import numpy as np

from .atmosphere import Body, load_body
from .simloop import SimulationLoop

CONCEPTUAL_DISCLAIMER = ("Simulador conceitual e educacional. Não é um simulador certificado (FSTD) "
                         "nem substitui dados do fabricante.")


class VehicleTemplate(ABC):
    """What a vehicle kind contributes to the shared engine."""

    id: str
    label: str  # shown in the UI / HUD (Portuguese)
    description: str
    telemetry_fields: Tuple[str, ...]

    @abstractmethod
    def default_body(self) -> Body:
        """Environment the template flies in when none is given."""

    @abstractmethod
    def build(self, body: Body | None = None, seed: int = 0, **options: Any) -> SimulationLoop:
        """A ready-to-run simulator (same seed -> same trajectory)."""

    @abstractmethod
    def default_guidance(self) -> Callable[[float, np.ndarray], Any]:
        """A simple command function for smoke tests and demos."""


class MarsDroneTemplate(VehicleTemplate):
    """The TCC Mars hexacopter (NASA MSH class) on the existing multirotor engine."""

    id = "drone_marte"
    label = "Drone Marte (hexacóptero classe NASA MSH)"
    description = "Hexacóptero do TCC em Marte: física 6-DoF, EKF e SADPF (fase 3)."
    telemetry_fields = ("t", "x", "y", "z", "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "power_w",
                        "thrust_total_n")
    vehicle_name = "mars_hexacopter_tcc"

    def default_body(self) -> Body:
        # operating-envelope wind (see tests/test_sadpf.py); the app's default Mars config is storm level
        return dataclasses.replace(load_body("mars"), wind_mean=5.0, wind_gust_std=2.0)

    def build(self, body: Body | None = None, seed: int = 0, **options: Any) -> SimulationLoop:
        from .simulator import TwinSimulator
        from .vehicle import load_vehicle
        return TwinSimulator(load_vehicle(self.vehicle_name), body or self.default_body(), seed=seed, **options)

    def default_guidance(self):
        from .simulator import hover_at
        return hover_at([0.0, 0.0, 10.0])


_REGISTRY: Dict[str, VehicleTemplate] = {}


def register(template: VehicleTemplate) -> VehicleTemplate:
    _REGISTRY[template.id] = template
    return template


register(MarsDroneTemplate())


def get_template(template_id: str) -> VehicleTemplate:
    if template_id not in _REGISTRY:
        raise KeyError(f"unknown vehicle template {template_id!r}; available: {sorted(_REGISTRY)}")
    return _REGISTRY[template_id]


def list_templates() -> List[VehicleTemplate]:
    return list(_REGISTRY.values())
