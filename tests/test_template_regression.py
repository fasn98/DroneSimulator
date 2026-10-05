"""
Regression: the Mars drone must fly exactly as before the VehicleTemplate abstraction.

tests/data/drone_regression_golden.npz was recorded by tools/make_regression_golden.py on the
commit before the refactor (same seeds, same cases); here the refactored engine reproduces it.
"""

import unittest
from pathlib import Path

import numpy as np

from src.physics.templates import get_template, list_templates
from tools.make_regression_golden import KEYS, cases

GOLDEN = Path(__file__).resolve().parent / "data" / "drone_regression_golden.npz"


class TestDroneRegression(unittest.TestCase):
    def test_trajectories_identical_to_pre_refactor(self):
        golden = np.load(GOLDEN)
        for name, sim, dur, fn in cases():
            tel = sim.run(dur, fn)
            for k in KEYS:
                np.testing.assert_allclose(tel.column(k), golden[f"{name}.{k}"], rtol=0, atol=1e-9,
                                           err_msg=f"{name}.{k}")
            if sim.sadpf is not None:
                np.testing.assert_array_equal(tel.column("sadpf_level"), golden[f"{name}.sadpf_level"])

    def test_template_builds_the_same_simulator(self):
        t = get_template("drone_marte")
        a = t.build(seed=5).run(8.0, t.default_guidance())
        b = t.build(seed=5).run(8.0, t.default_guidance())
        np.testing.assert_array_equal(a.column("z"), b.column("z"))
        self.assertIn("drone_marte", [x.id for x in list_templates()])


if __name__ == "__main__":
    unittest.main()
