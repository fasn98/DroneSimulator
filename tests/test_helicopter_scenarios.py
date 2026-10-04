"""
Passo 2 validation: scenarios, engine failures and SADPF of the "Helicóptero UTI" template (classe H135).

The criteria are those of the model (docs/helicoptero-uti.md, "Passo 2"); regulatory numbers used as
criteria (100 ft/min at VTOSS, 15 ft above the take-off surface) are linked to the official text there.
"""

import unittest

import numpy as np

from src.physics.helicopter import HelicopterSimulator
from src.physics.helicopter.procedures import KT, min_descent_speed, vtoss
from src.physics.helicopter.scenarios import CatAConfig, autorotation, cat_a_run, rescue, transfer

HOT_HIGH = CatAConfig(elevation_m=1500.0, delta_t=25.0)


class TestSadpf(unittest.TestCase):
    def test_no_false_alarm_in_normal_flight(self):
        r = transfer(distance_m=2500.0, cruise_alt=150.0, duration=200.0)
        self.assertTrue(r["landing"]["landed"])
        self.assertEqual(r["sim"].sadpf.level, 0)
        self.assertLess(r["final_position_error_m"], 5.0)

    def test_single_failure_in_cruise_recommends_continue_oei(self):
        sim = HelicopterSimulator(wind=False, start_position=(0.0, 0.0, 300.0), start_velocity=(100 * KT, 0, 0),
                                  sadpf=True)
        state = {"done": False}

        def g(t, x):
            if t >= 2.0 and not state["done"]:
                state["done"] = True
                sim.fail_engine(1)
            return np.array([x[0], 0.0, 300.0]), np.array([100 * KT, 0.0, 0.0]), 0.0
        sim.run(8.0, g, 0.1)
        s = sim.sadpf
        self.assertEqual(s.level, 3)
        self.assertEqual(s.failed, [False, True])  # the right engine isolated
        self.assertLess(s.detect_t - 2.0, 1.5)
        self.assertEqual(s.recommendation_code, "prosseguir_oei")


class TestCategoryA(unittest.TestCase):
    def test_reject_before_tdp_and_continue_after_tdp_light(self):
        r = cat_a_run(2500.0, "reject")
        self.assertEqual(r["sadpf"]["recommendation"], "abortar")
        self.assertLess(r["sadpf"]["detection_delay_s"], 1.5)
        self.assertTrue(r["safe"], r["reason"])
        c = cat_a_run(2500.0, "continue")
        self.assertEqual(c["sadpf"]["recommendation"], "prosseguir")
        self.assertTrue(c["safe"], c["reason"])
        self.assertGreaterEqual(c["max_oei_rating"], 1)  # OEI 30 s rating used

    def test_mass_limit_hot_and_high(self):
        # below the computed Cat A mass (2 614 kg for 1 500 m, ISA+25, no wind) both branches are safe,
        # above it the continue branch is not (the limit the scenario demonstrates), and at MTOW the aircraft
        # cannot even reach the TDP with both engines
        light = cat_a_run(2550.0, "continue", HOT_HIGH)
        self.assertTrue(light["safe"], light["reason"])
        heavy = cat_a_run(2700.0, "continue", HOT_HIGH)
        self.assertFalse(heavy["safe"])
        self.assertGreater(heavy["max_height_loss_m"], light["max_height_loss_m"])
        self.assertTrue(cat_a_run(2700.0, "reject", HOT_HIGH)["safe"])
        mtow = cat_a_run(2980.0, "continue", HOT_HIGH)
        self.assertFalse(mtow["safe"])
        self.assertIn("TDP", mtow["reason"])

    def test_vtoss_meets_100_fpm(self):
        sim = HelicopterSimulator(wind=False)
        v = vtoss(sim.p, sim.atm, 0.0, sim.p.mass)
        self.assertIsNotNone(v)

    def test_deterministic(self):
        a = cat_a_run(2500.0, "continue", seed=3)
        b = cat_a_run(2500.0, "continue", seed=3)
        np.testing.assert_array_equal(a["telemetry"].column("z"), b["telemetry"].column("z"))


class TestAutorotation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fwd = autorotation("forward", height=150.0)
        cls.vert = autorotation("vertical", height=150.0)

    def test_forward_descent_rate_matches_energy_method(self):
        f = self.fwd
        self.assertEqual(f["sadpf"]["recommendation"], "autorrotacao")
        self.assertLess(abs(f["rod_steady_ms"] / f["rod_predicted_ms"] - 1.0), 0.15)
        self.assertGreaterEqual(f["min_nr_before_cushion_pct"], 85.0)  # power-off NR limit (TCDS)
        self.assertLessEqual(f["max_nr_pct"], 106.0)

    def test_forward_flare_lands(self):
        land = self.fwd["landing"]
        self.assertTrue(land["landed"])
        self.assertLessEqual(land["touchdown_sink_ms"], 2.5)
        self.assertLessEqual(land["touchdown_ground_speed_ms"], 15.0)
        self.assertLess(land["max_tilt_after_deg"], 20.0)

    def test_vertical_is_worse(self):
        self.assertGreater(self.vert["rod_steady_ms"], 1.8 * self.fwd["rod_steady_ms"])
        self.assertGreater(self.vert["landing"]["touchdown_sink_ms"], 3 * self.fwd["landing"]["touchdown_sink_ms"])

    def test_min_descent_speed_between_vbe_and_vbr(self):
        sim = HelicopterSimulator(wind=False)
        md = min_descent_speed(sim.p, sim.atm, 0.0, sim.p.mass)
        self.assertTrue(55.0 <= md["v_md_kt"] <= 85.0, md)


class TestRescue(unittest.TestCase):
    def test_restricted_area_crosswind_ground_effect(self):
        r = rescue()
        self.assertTrue(r["landing"]["landed"])
        self.assertLess(r["final_position_error_m"], 5.0)
        self.assertLessEqual(r["landing"]["touchdown_sink_ms"], 1.5)
        self.assertLess(r["k_ge_hover"], 1.0)
        self.assertLess(r["power_hover_kw"], r["power_oge_same_wind_kw"])


if __name__ == "__main__":
    unittest.main()
