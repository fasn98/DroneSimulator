"""
Passo 3 (aval): "previsão de ramos", advisory function of the SADPF (consultivo, conceitual, não certificado),
and the take-off mass limit at the HEMS operating site applied to the radius of action.
"""

import unittest

from src.physics.helicopter.advisory import BUDGET_S, LABEL
from src.physics.helicopter.mission import catA_fuel_and_radius
from src.physics.helicopter.scenarios import cat_a_run, rescue_site_config

# rescue site at 1.000 m ISA+20, 2 600,7 kg (above the TDP-12 m site limit): an alert case of the 130-case check
# with the corrected touchdown and the default reject procedure v2 (docs/helicoptero/recalculo_abortar_v2.json):
# failure 7 m above the site (5 m below the TDP) -> the procedure says abortar, which touches down too hard;
# prosseguir is safe. Failure 3 m above the site (9 m below the TDP): abortar safe, no alert.
CFG = rescue_site_config(1000.0, 20.0, 0.0)
MASS = 2600.7
ALERT_REL, QUIET_REL = -5.0, -9.0


class TestBranchPrediction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.shown = cat_a_run(MASS, cfg=CFG, fail_rel_tdp_m=ALERT_REL, advisory=True)  # procedure flown, alert shown
        cls.followed = cat_a_run(MASS, cfg=CFG, fail_rel_tdp_m=ALERT_REL, follow_advisory=True)
        cls.quiet = cat_a_run(MASS, cfg=CFG, fail_rel_tdp_m=QUIET_REL, advisory=True)

    def test_alert_when_procedure_branch_predicted_unsafe(self):
        a = self.shown["advisory"]
        self.assertEqual(a["label"], LABEL)
        self.assertIn("não certificado", a["label"])
        self.assertEqual(a["procedure_action"], "abortar")  # failure recognised before the TDP
        self.assertTrue(a["alert"])
        self.assertEqual(a["advise"], "prosseguir")
        self.assertFalse(a["predicted"]["abortar"]["safe"])
        self.assertTrue(a["predicted"]["prosseguir"]["safe"])

    def test_procedure_stays_default(self):
        # only displayed: the procedure branch is still flown, and it fails (the prediction was right)
        self.assertEqual(self.shown["branch"], "reject")
        self.assertFalse(self.shown["safe"])

    def test_following_the_advisory_changes_the_outcome(self):
        self.assertEqual(self.followed["branch"], "continue")
        self.assertTrue(self.followed["safe"])

    def test_no_alert_when_procedure_branch_safe(self):
        a = self.quiet["advisory"]
        self.assertEqual(a["procedure_action"], "abortar")
        self.assertFalse(a["alert"])
        self.assertIsNone(a["advise"])

    def test_wall_time_measured(self):
        # the 1 s budget is reported by tools/heli_step3c_report.py; here only a loose bound (shared CI runners)
        for r in (self.shown, self.quiet):
            a = r["advisory"]
            self.assertGreater(a["wall_time_s"], 0.0)
            self.assertLess(a["wall_time_s"], 5.0 * BUDGET_S)
            self.assertEqual(a["within_budget"], a["wall_time_s"] <= BUDGET_S)


class TestRescueSiteMassLimit(unittest.TestCase):
    def test_site_limit_in_radius(self):
        lim = 2516.875  # TDP 12 m, 1.500 m ISA+25 (docs/helicoptero/passo3c_resultados.json)
        r = catA_fuel_and_radius(2614.0, "resgate", site_limit_kg=lim)
        free = catA_fuel_and_radius(2614.0, "resgate")
        # at the maximum radius the return take-off is below the limit: radius unchanged
        self.assertLessEqual(r["return_takeoff_mass_kg"], lim)
        self.assertAlmostEqual(r["radius_nm"], free["radius_nm"], places=3)
        # fuel aboard at the site limited, so short missions with a full tank are not allowed
        self.assertAlmostEqual(r["site_fuel_max_kg"], lim - r["zero_fuel_mass_back_kg"])
        self.assertGreater(r["return_takeoff_mass_max_kg"], lim)
        self.assertGreater(r["site_min_radius_full_fuel_nm"], 30.0)
        self.assertLess(r["site_min_radius_full_fuel_nm"], r["radius_nm"])

    def test_site_limit_reduces_fuel_when_binding(self):
        lim = 2300.0  # below the return take-off at the maximum radius: the origin fuel must be reduced
        r = catA_fuel_and_radius(2614.0, "resgate", site_limit_kg=lim)
        free = catA_fuel_and_radius(2614.0, "resgate")
        self.assertEqual(r["fuel_limited_by"], "massa no local de resgate")
        self.assertLessEqual(r["return_takeoff_mass_kg"], lim + 0.5)
        self.assertLess(r["radius_nm"], free["radius_nm"])


if __name__ == "__main__":
    unittest.main()
