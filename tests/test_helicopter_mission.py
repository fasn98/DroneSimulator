"""
Passo 3 validation: UTI interior, mass and balance, fuel model and radius of action (Helicóptero UTI, classe H135).
"""

import unittest

from src.physics.helicopter import HeliParams, calibrate_drag_area
from src.physics.helicopter.isa import HeliAtmosphere
from src.physics.helicopter.mission import (EQUIPMENT, FUEL_CAPACITY_KG, RESERVE_MIN, Loading, catA_fuel_and_radius,
                                            cg_limits, radius_of_action)
from src.physics.helicopter.rotor import _fuel_mission, fuel_flow_params


def params():
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    p.ff_idle_kgh, p.sfc_marginal = fuel_flow_params(p)
    return p


class TestFuelModel(unittest.TestCase):
    def test_reproduces_published_endurance_and_range(self):
        p = params()
        self.assertAlmostEqual(_fuel_mission(p, p.ff_idle_kgh, p.sfc_marginal, "be", 560.0)[0], 3.6, delta=0.05)
        self.assertAlmostEqual(_fuel_mission(p, p.ff_idle_kgh, p.sfc_marginal, "br", 560.0)[1], 342.0, delta=4.0)
        self.assertGreater(p.ff_idle_kgh, 0.0)
        self.assertGreater(p.sfc_marginal, 0.0)


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
        L = Loading()
        r = catA_fuel_and_radius(L.zero_fuel_mass + 300.0, HeliAtmosphere(), L, p)
        self.assertAlmostEqual(r["fuel_kg"], 300.0)
        self.assertEqual(r["fuel_limited_by"], "massa Cat A")
        full = catA_fuel_and_radius(2980.0, HeliAtmosphere(), L, p)
        self.assertEqual(full["fuel_limited_by"], "tanque")
        self.assertGreater(full["radius_nm"], r["radius_nm"])
        self.assertTrue(full["cg_takeoff"]["inside"] and full["cg_reserve_only"]["inside"])

    def test_reserve_is_20_min(self):
        self.assertEqual(RESERVE_MIN, 20.0)  # 14 CFR 135.209(b)
        p = params()
        r = radius_of_action(400.0, Loading().zero_fuel_mass, HeliAtmosphere(), p)
        self.assertGreater(r.reserve_kg, 0.0)
        self.assertGreater(r.radius_nm, 0.0)


if __name__ == "__main__":
    unittest.main()
