"""
Twin v2 physics core.

Self-contained (numpy + scipy only): planetary atmospheres, density-dependent
rotors, chemical thrusters, quaternion 6-DoF rigid-body dynamics with RK4,
ground contact, stochastic wind, geometric flight control and sizing tools.

Quick start:
    from src.physics import load_vehicle, load_body, TwinSimulator, hover_at
    sim = TwinSimulator(load_vehicle("mars_hexacopter_tcc"), load_body("mars"))
    tel = sim.run(60.0, hover_at([0, 0, 10]))
"""

from .atmosphere import Body, body_from_config, load_body
from .actuators import Rotor, Thruster
from .vehicle import Vehicle, load_vehicle, list_vehicles
from .dynamics import RigidBodyDynamics, quat_from_euler, euler_from_quat
from .control import GeometricController, Gains
from .simulator import TwinSimulator, hover_at, waypoint_route
from .sizing import hover_report, max_hover_mass, ideal_hover_power, lunar_delta_v

__all__ = [
    "Body", "body_from_config", "load_body", "Rotor", "Thruster", "Vehicle", "load_vehicle", "list_vehicles",
    "RigidBodyDynamics", "quat_from_euler", "euler_from_quat", "GeometricController", "Gains",
    "TwinSimulator", "hover_at", "waypoint_route", "hover_report", "max_hover_mass",
    "ideal_hover_power", "lunar_delta_v",
]
