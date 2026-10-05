"""Records reference trajectories of the Mars drone (fixed seeds) for tests/test_template_regression.py.

Run once on the code BEFORE a refactor; the test then checks that the refactored engine reproduces them.
"""
import dataclasses
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.physics import RotorFault, SensorConfig, TwinSimulator, hover_at, load_body, load_vehicle, waypoint_route  # noqa: E402

KEYS = ("t", "x", "y", "z", "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "power_w", "thrust_total_n")


def cases():
    mars = dataclasses.replace(load_body("mars"), wind_mean=5.0, wind_gust_std=2.0)
    hexa = load_vehicle("mars_hexacopter_tcc")
    yield "plain_route", TwinSimulator(hexa, mars, seed=3), 30.0, waypoint_route([[0, 0, 8], [15, 5, 8]], cruise_speed=3.0, max_accel=1.0)
    yield "sadpf_rotor_fault", TwinSimulator(hexa, mars, seed=2, sensors=SensorConfig(), sadpf=True,
                                             faults=[RotorFault(2, 0.5, 15.0)]), 30.0, hover_at([0, 0, 10])


def record(path: Path):
    out = {}
    for name, sim, dur, fn in cases():
        tel = sim.run(dur, fn)
        for k in KEYS:
            out[f"{name}.{k}"] = tel.column(k)
        if sim.sadpf is not None:
            out[f"{name}.sadpf_level"] = tel.column("sadpf_level")
    np.savez_compressed(path, **out)


if __name__ == "__main__":
    record(Path(__file__).resolve().parents[1] / "tests" / "data" / "drone_regression_golden.npz")
