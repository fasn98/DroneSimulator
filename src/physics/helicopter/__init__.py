"""
Template "Helicóptero UTI": light twin EMS helicopter, classe H135 (generic, no manufacturer branding).

Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.

    from src.physics.helicopter import HelicopterSimulator, EngineFault
    from src.physics import hover_at
    sim = HelicopterSimulator(seed=1)
    tel = sim.run(30.0, hover_at([0, 0, 15], climb_rate=2.0))
"""

from .isa import HeliAtmosphere
from .params import TABLE, HeliParams
from .rotor import best_speeds, calibrate_drag_area, level_flight, main_rotor, power_curve
from .simulator import EngineFault, HelicopterSimulator, SasGains, scripted

__all__ = ["HeliAtmosphere", "TABLE", "HeliParams", "best_speeds", "calibrate_drag_area", "level_flight",
           "main_rotor", "power_curve", "EngineFault", "HelicopterSimulator", "SasGains", "scripted"]
