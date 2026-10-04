"""
Passo 1 validation: physics core of the "Helicóptero UTI" template (classe H135).

Reference values are public (src/physics/helicopter/params.py, docs/fontes.md); where the model
is checked against a derived reference, the derivation is in the test and in docs/helicoptero-uti.md.
"""

import math
import unittest

import numpy as np

from src.physics.helicopter import (EngineFault, HeliAtmosphere, HeliParams, HelicopterSimulator, best_speeds,
                                    calibrate_drag_area, level_flight, main_rotor, scripted)
from src.physics.helicopter.isa import density_altitude, isa_density
from src.physics.helicopter.model import engine_limit_w
from src.physics.helicopter.rotor import ground_effect_factor
from src.physics.simulator import hover_at

FT = 0.3048


def params():
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    return p


def hold(pos, yaw=0.0):
    pos = np.asarray(pos, float)
    return lambda t, x: (pos, np.zeros(3), yaw)


class TestAtmosphere(unittest.TestCase):
    def test_isa_and_density_altitude(self):
        self.assertAlmostEqual(isa_density(0.0), 1.225, places=3)
        self.assertAlmostEqual(isa_density(11000 * 0.0 + 3000.0), 0.9093, places=3)  # ISA table, 3 km
        hot = HeliAtmosphere(elevation_m=1000.0, delta_t=20.0)
        self.assertGreater(hot.density_altitude(0.0), 1000.0 + 600.0)  # ~ +120 ft per degC above ISA


class TestHover(unittest.TestCase):
    def setUp(self):
        self.p = params()

    def test_hover_power_and_figure_of_merit(self):
        h = level_flight(self.p, 0.0, 1.225)
        fm = h.thrust ** 1.5 / math.sqrt(2 * 1.225 * self.p.area) / h.p_main
        self.assertGreater(fm, 0.65)
        self.assertLess(fm, 0.80)
        # tail rotor power is computed (torque balance), not assumed; expected order ~10 % of the main rotor
        self.assertGreater(h.p_tail / h.p_main, 0.05)
        self.assertLess(h.p_tail / h.p_main, 0.20)
        # AEO take-off power (2 x 75 % torque) must cover hover OGE at MTOW, sea level ISA
        self.assertLess(h.p_engines, self.p.gearbox_limit_kw("TO") * 1e3)

    def test_hover_ceiling_matches_published_hoge(self):
        """Derived reference: at the published HOGE ceiling (7,200 ft ISA, Airbus H135 data) the hover power
        required equals the take-off power available. Model within +-15 % (divergence documented)."""
        p = self.p
        h_m = 7200 * FT
        rho = isa_density(h_m)
        p_req = level_flight(p, 0.0, rho).p_engines
        p_av = 2 * engine_limit_w(p, HeliAtmosphere(), h_m, "TO", 2)
        self.assertLess(abs(p_req / p_av - 1.0), 0.15, f"required {p_req / 1e3:.0f} kW, available {p_av / 1e3:.0f} kW")


class TestGroundEffect(unittest.TestCase):
    def test_cheeseman_bennett_factor(self):
        R = 5.1
        self.assertAlmostEqual(ground_effect_factor(R, R, 0.0, 12.0), 1 - 1 / 16)
        self.assertEqual(ground_effect_factor(math.inf, R, 0.0, 12.0), 1.0)
        # limited at z/R = 0.5 (formula singular at 0.25)
        self.assertAlmostEqual(ground_effect_factor(0.1, R, 0.0, 12.0), 1 - 0.25)
        # fades with forward speed
        self.assertGreater(ground_effect_factor(R, R, 20.0, 12.0), ground_effect_factor(R, R, 0.0, 12.0))

    def test_ground_effect_reduces_power(self):
        p = params()
        oge = level_flight(p, 0.0, 1.225).p_engines
        ige = level_flight(p, 0.0, 1.225, z_hub=3.0).p_engines
        self.assertLess(ige, 0.95 * oge)


class TestForwardFlight(unittest.TestCase):
    def test_power_curve_bucket(self):
        p = params()
        b = best_speeds(p, 1.225)
        p0 = level_flight(p, 0.0, 1.225).p_engines / 1e3
        p140 = level_flight(p, 140 * 0.514444, 1.225).p_engines / 1e3
        self.assertLess(b["v_be_kt"], b["v_br_kt"])
        self.assertLess(b["p_be_kw"], 0.7 * p0)
        self.assertGreater(p140, b["p_be_kw"])
        # drag area calibrated so the published fast cruise (136 kt) needs AEO max-continuous power
        self.assertAlmostEqual(level_flight(p, 136 * 0.514444, 1.225).p_engines / 1e3, p.gearbox_limit_kw("MCP"),
                               delta=0.5)


class TestVortexRing(unittest.TestCase):
    def test_vrs_flag_in_vertical_descent_only(self):
        p = params()
        om = p.omega100
        th = math.radians(8.0)
        vh = main_rotor(p, th, 0.0, 0.0, 1.225, om).v_h
        self.assertTrue(main_rotor(p, th, -1.0 * vh, 0.0, 1.225, om).vrs)
        self.assertFalse(main_rotor(p, th, -0.2 * vh, 0.0, 1.225, om).vrs)
        self.assertFalse(main_rotor(p, th, -1.0 * vh, 2.0 * vh, 1.225, om).vrs)

    def test_inflow_is_single_valued(self):
        p = params()
        om = p.omega100
        for th in (-2.0, 0.0, 3.0, 10.0):
            for vc in (0.0, -5.0, -15.0, -25.0):
                a = main_rotor(p, math.radians(th), vc, 0.0, 1.225, om, lam_i0=-0.2).power
                b = main_rotor(p, math.radians(th), vc, 0.0, 1.225, om, lam_i0=0.2).power
                self.assertAlmostEqual(a, b, delta=1.0)


class TestRotorSpeed(unittest.TestCase):
    def test_nr_decays_without_power(self):
        """Both engines out, collective held (no pilot action): NR must fall below the power-off minimum."""
        sim = HelicopterSimulator(wind=False, start_position=(0, 0, 500.0),
                                  engine_faults=[EngineFault(0, 2.0), EngineFault(1, 2.0)])
        sim.collective_hold = sim.u.collective  # trimmed hover collective, held: no pilot reaction
        tel = sim.run(6.0, hold([0, 0, 500.0]), record_every=0.05)
        nr = tel.column("nr_pct")
        t = tel.column("t")
        self.assertGreater(nr[t < 2.0].min(), 99.0)
        self.assertLess(nr[-1], 85.0)
        self.assertTrue(np.all(np.diff(nr[t > 3.0]) < 0))  # monotonic decay once the engines are down

    def test_autorotation_holds_nr_in_power_off_band(self):
        sim = HelicopterSimulator(wind=False, start_position=(0, 0, 600.0),
                                  engine_faults=[EngineFault(0, 2.0), EngineFault(1, 2.0)])
        fn = scripted(hold([0, 0, 600.0]), 3.0, lambda: sim.set_autorotation(True))
        tel = sim.run(20.0, fn, record_every=0.1)
        t, nr, vz = tel.column("t"), tel.column("nr_pct"), tel.column("vz")
        steady = t > 10.0
        self.assertGreaterEqual(nr[steady].min(), 85.0)
        self.assertLessEqual(nr[steady].max(), 106.0)
        self.assertLess(vz[steady].mean(), -10.0)  # descending, rotor driven by the air
        self.assertTrue(np.all(tel.column("p_eng1_kw")[steady] < 1.0))


class TestEngines(unittest.TestCase):
    def test_oei_ratings_follow_the_timer(self):
        sim = HelicopterSimulator(wind=False, start_position=(0, 0, 300.0), engine_faults=[EngineFault(1, 1.0)])
        tel = sim.run(35.0, hold([0, 0, 300.0]), record_every=0.5)
        t, rating = tel.column("t"), tel.column("rating")
        self.assertTrue(np.all(rating[(t > 2) & (t < 30)] == 1))  # OEI 30 s
        self.assertTrue(np.all(rating[t > 32] == 2))  # OEI 2 min
        p = sim.p
        self.assertAlmostEqual(tel.column("p_avail_kw")[(t > 2) & (t < 30)].max(), p.gearbox_limit_kw("OEI30"), delta=1.0)
        self.assertLess(tel.column("p_eng2_kw")[-1], 1.0)


class TestDeterminism(unittest.TestCase):
    def test_same_seed_same_trajectory(self):
        atm = HeliAtmosphere(wind_mean=6.0, wind_gust_std=2.0)
        runs = [HelicopterSimulator(atmosphere=atm, seed=s).run(12.0, hover_at([0, 0, 15], climb_rate=2.0))
                for s in (4, 4, 5)]
        np.testing.assert_array_equal(runs[0].column("y"), runs[1].column("y"))
        self.assertGreater(np.max(np.abs(runs[0].column("y") - runs[2].column("y"))), 1e-6)


class TestTakeoffAndHover(unittest.TestCase):
    def test_takeoff_climb_hover_keeps_nr(self):
        sim = HelicopterSimulator(wind=False)
        tel = sim.run(30.0, hover_at([0, 0, 15], climb_rate=2.0), record_every=0.2)
        t = tel.column("t")
        self.assertGreater(tel.column("nr_pct").min(), 97.0)  # power-on minimum, TCDS
        self.assertLess(abs(tel.column("z")[-1] - 15.0), 0.5)
        self.assertLess(np.max(np.hypot(tel.column("x"), tel.column("y"))[t > 22]), 3.0)


if __name__ == "__main__":
    unittest.main()


class TestAntiTorque(unittest.TestCase):
    def test_duct_and_fin(self):
        from dataclasses import replace
        p = params()
        hover = level_flight(p, 0.0, 1.225)
        self.assertEqual(hover.f_fin, 0.0)  # no fin force in hover
        open_rotor = level_flight(replace(p, tr_sigma_d=0.5), 0.0, 1.225)
        # ideal duct, sigma_d = 1: induced power 1/sqrt(2) of the open rotor at the same thrust
        tr_prof = open_rotor.p_tail - (open_rotor.p_tail - hover.p_tail) / (1 - 2 ** -0.5)
        self.assertGreater(open_rotor.p_tail, hover.p_tail * 1.3)
        cruise = level_flight(p, 136 * 0.514444, 1.225)
        no_fin = level_flight(replace(p, fin_S=0.0), 136 * 0.514444, 1.225)
        self.assertGreater(cruise.f_fin, 0.0)
        self.assertLess(cruise.t_tail, no_fin.t_tail)  # the fin unloads the Fenestron
        self.assertLess(cruise.p_tail, no_fin.p_tail)
        self.assertAlmostEqual(cruise.t_tail + cruise.f_fin, no_fin.t_tail, delta=1.0)  # same anti-torque
        self.assertGreater(tr_prof, 0.0)
