"""
Passo 3 validation: UTI interior, mass and balance, fuel model and radius of action (Helicóptero UTI, classe H135).
"""

import unittest

from src.physics.helicopter import HeliParams, calibrate_drag_area
from src.physics.helicopter.mission import (EQUIPMENT, FUEL_CAPACITY_KG, KT, PROFILES, REF_ATM, REF_CRUISE_ALT_M,
                                            RESERVE_MIN, Loading, catA_fuel_and_radius, cg_limits, fuel_flow_kg_s,
                                            loading_for, radius_of_action)
from src.physics.helicopter.rotor import _fuel_mission, best_speeds, fuel_flow_params, level_flight


def params():
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    p.ff_idle_kgh, p.sfc_marginal = fuel_flow_params(p)
    return p


class TestFuelModel(unittest.TestCase):
    def test_close_to_published_endurance_and_range(self):
        p = params()
        self.assertAlmostEqual(_fuel_mission(p, p.ff_idle_kgh, p.sfc_marginal, "be", 560.0)[0] / 3.6, 1.0, delta=0.03)
        self.assertAlmostEqual(_fuel_mission(p, p.ff_idle_kgh, p.sfc_marginal, "br", 560.0)[1] / 342.0, 1.0,
                               delta=0.03)

    def test_specific_range_falls_with_mass(self):
        p = params()
        rho = REF_ATM.density(REF_CRUISE_ALT_M)
        sr = []
        for m in (2100.0, 2500.0, 2900.0):
            v = best_speeds(p, rho, m)["v_br_kt"] * KT
            sr.append(v / fuel_flow_kg_s(p, level_flight(p, v, rho, m).p_engines))
        self.assertGreater(sr[0], sr[1])
        self.assertGreater(sr[1], sr[2])


class TestMassAndBalance(unittest.TestCase):
    def test_budget_and_envelope(self):
        L = Loading()
        self.assertAlmostEqual(L.zero_fuel_mass, sum(i.mass_kg for i in L.items))
        self.assertTrue(all(i.status in ("FONTE", "DERIVADO", "ESTIMADO") for i in L.items))
        for fuel in (0.0, 50.0, FUEL_CAPACITY_KG):
            c = L.cg(fuel)
            self.assertTrue(c["inside"], c)
        fwd, aft = cg_limits(3175.0)
        self.assertAlmostEqual(fwd, 4237.5)
        self.assertAlmostEqual(aft, 4349.0)

    def test_equipment_sources(self):
        fonte = [i for i in EQUIPMENT if i.status == "FONTE"]
        self.assertTrue(all("http" in i.source for i in fonte))


class TestRadius(unittest.TestCase):
    def test_fuel_limited_by_cat_a_mass(self):
        p = params()
        zfm = loading_for(True).zero_fuel_mass
        r = catA_fuel_and_radius(zfm + 300.0, "conservador", p)
        self.assertAlmostEqual(r["fuel_kg"], 300.0)
        self.assertEqual(r["fuel_limited_by"], "massa Cat A")
        full = catA_fuel_and_radius(2980.0, "conservador", p)
        self.assertEqual(full["fuel_limited_by"], "tanque")
        self.assertGreater(full["radius_nm"], r["radius_nm"])
        for k in ("cg_takeoff", "cg_return_takeoff", "cg_reserve_only"):
            self.assertTrue(full[k]["inside"], k)

    def test_cruise_condition_is_common(self):
        # the take-off condition only sets the fuel: same fuel -> same radius whatever the condition
        p = params()
        a = catA_fuel_and_radius(2980.0, "resgate", p)
        b = catA_fuel_and_radius(2809.0, "resgate", p)  # tank-limited too
        self.assertAlmostEqual(a["radius_nm"], b["radius_nm"], delta=0.01)

    def test_profiles(self):
        p = params()
        r = {k: catA_fuel_and_radius(2980.0, k, p) for k in PROFILES}
        self.assertLessEqual(r["conservador"]["radius_nm"], r["resgate"]["radius_nm"])
        self.assertLessEqual(r["conservador"]["radius_nm"], r["transferencia"]["radius_nm"])
        self.assertLess(r["resgate"]["zero_fuel_mass_out_kg"], r["resgate"]["zero_fuel_mass_back_kg"])
        self.assertGreater(r["transferencia"]["zero_fuel_mass_out_kg"], r["transferencia"]["zero_fuel_mass_back_kg"])

    def test_reserve_is_20_min(self):
        self.assertEqual(RESERVE_MIN, 20.0)  # RBAC 91.151(b); 14 CFR 135.209(b)
        r = radius_of_action(400.0, Loading().zero_fuel_mass, p=params())
        self.assertGreater(r.reserve_kg, 0.0)
        self.assertGreater(r.radius_nm, 0.0)


class TestRescueSite(unittest.TestCase):
    def test_return_takeoff_masses(self):
        r = catA_fuel_and_radius(2980.0, "resgate", params())
        self.assertGreater(r["return_takeoff_mass_max_kg"], r["return_takeoff_mass_kg"])
        self.assertLess(r["return_takeoff_mass_max_kg"], r["zero_fuel_mass_back_kg"] + r["fuel_kg"])

    def test_site_takeoff_sea_level(self):
        from src.physics.helicopter.scenarios import rescue_site_takeoff
        r = rescue_site_takeoff(2369.0, heights=(3.0, 9.0))
        self.assertGreater(r["aeo_margin_ige_kw"], r["aeo_margin_oge_kw"])
        self.assertGreater(r["aeo_margin_oge_kw"], 0.0)
        self.assertIsNone(r["exposure"])
        low, mid = r["sweep"]
        self.assertTrue(low["reject"]["safe"])
        self.assertFalse(low["continue"]["safe"])  # too low to continue (29.59(c))
        self.assertTrue(mid["reject"]["safe"] and mid["continue"]["safe"])


if __name__ == "__main__":
    unittest.main()
