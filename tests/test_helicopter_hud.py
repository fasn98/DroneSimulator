"""
Passo 4 validation: HUD scenes of the Helicóptero UTI (web/heli/) and the data they replay
(web/heli/data/*.json, written by tools/heli_scenes_export.py from the Twin), plus the parallel branch prediction.
"""

import json
import os
import re
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "web", "heli")
DISCLAIMER = "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante."
FOOTER = "voo, falha e respostas calculados pelo Twin (não é animação)"


def data(name):
    with open(os.path.join(WEB, "data", f"{name}.json"), encoding="utf-8") as f:
        return json.load(f)


class TestHudPage(unittest.TestCase):
    def test_disclaimer_and_footer_always_in_the_page(self):
        html = open(os.path.join(WEB, "index.html"), encoding="utf-8").read()
        self.assertIn(DISCLAIMER, html)
        self.assertIn(FOOTER, html)
        # the disclaimer and the footer are outside the per-scene panels (never hidden by a scene)
        js = open(os.path.join(WEB, "heli.js"), encoding="utf-8").read()
        self.assertNotIn("'disclaimer').style.display", js)
        self.assertNotIn("'foot').style.display", js)
        for s in ("consultivo · conceitual · não certificado", "medido em ambiente de desenvolvimento"):
            self.assertIn(s, (html + js).lower())

    def test_no_manufacturer_marks(self):
        pat = re.compile(r"airbus|eurocopter|helibras|\bEC ?135\b|sikorsky|black ?hawk", re.I)
        for fn in ("index.html", "heli.js", "heli.css"):
            self.assertIsNone(pat.search(open(os.path.join(WEB, fn), encoding="utf-8").read()), fn)
        for name in ("cat_a", "advisory", "autorotation", "mission"):
            txt = open(os.path.join(WEB, "data", f"{name}.json"), encoding="utf-8").read()
            self.assertIsNone(pat.search(txt), name)

    def test_three_js_vendored_with_license(self):
        for fn in ("three.module.min.js", "three.core.min.js", "LICENSE-three.txt"):
            self.assertTrue(os.path.exists(os.path.join(WEB, "vendor", fn)), fn)


class TestSceneData(unittest.TestCase):
    def test_cat_a_both_branches(self):
        d = data("cat_a")
        self.assertEqual(set(d["runs"]), {"reject", "continue"})
        r, c = d["runs"]["reject"], d["runs"]["continue"]
        self.assertTrue(r["evaluation"]["safe"], r["evaluation"].get("reason"))
        self.assertTrue(c["evaluation"]["safe"], c["evaluation"].get("reason"))
        self.assertEqual(c["evaluation"]["criterion"], "elevado_29_60")
        self.assertIn("literal_29_59c", c)  # the literal reading shown side by side
        self.assertGreaterEqual(c["evaluation"]["max_drop_below_deck_m"], 0.0)
        m = d["cat_a_mass"]
        self.assertLessEqual(m["g115_kg"], m["g126_kg"])  # range by the duct gain
        self.assertLess(m["literal_29_59c_kg"], m["g115_kg"])
        self.assertAlmostEqual(d["mass_kg"], m["g126_kg"])
        # frames: same length in every column, OEI rating after the failure, failed engine torque near zero
        f = c["frames"]
        n = len(f["t"])
        self.assertTrue(all(len(v) == n for v in f.values()))
        k = next(i for i, t in enumerate(f["t"]) if t > c["fail_time"] + 4.0)
        self.assertGreaterEqual(f["rating"][k], 1)
        self.assertLess(f["tq1_pct"][k], 5.0)
        self.assertGreater(f["tq2_pct"][k], 50.0)
        self.assertLessEqual(c["torque_limits_pct"]["OEI2"], c["torque_limits_pct"]["OEI30"])

    def test_advisory_alert_scene(self):
        d = data("advisory")
        p, s = d["runs"]["procedimento"], d["runs"]["consultivo_seguido"]
        a = p["advisory"]
        self.assertTrue(a["alert"])
        self.assertEqual(a["procedure_action"], "prosseguir")
        self.assertEqual(a["advise"], "abortar")
        self.assertIn("não certificado", a["label"])
        self.assertEqual(a["mode"], "paralelo")
        self.assertLessEqual(a["wall_time_s"], 1.0)
        self.assertFalse(p["evaluation"]["safe"])  # the procedure (default) as flown
        self.assertTrue(s["evaluation"]["safe"])  # following the advisory
        self.assertGreater(d["mass_kg"], d["site_limit_kg"])  # demonstration case, stated in the scene

    def test_autorotation_comparison(self):
        d = data("autorotation")
        v, f = d["runs"]["vertical"], d["runs"]["forward"]
        self.assertTrue(v["landing"]["landed"] and f["landing"]["landed"])
        self.assertGreater(v["rod_steady_ms"], 1.8 * f["rod_steady_ms"])
        self.assertGreater(v["landing"]["touchdown_sink_ms"], f["landing"]["touchdown_sink_ms"])
        # the known flare limitation is visible in the data and stated in the scene
        self.assertGreater(f["landing"]["touchdown_ground_speed_ms"] / 0.514444, d["goal"]["ground_speed_kt"])
        self.assertIn("flare", d["flare_limitation"].lower())

    def test_mission_panel(self):
        d = data("mission")
        self.assertEqual(set(d["profiles"]), {"resgate", "transferencia", "conservador"})
        hot = next(c for c in d["conditions"] if c["condition"] == "1.500 m, ISA+25")
        r = hot["profiles"]["resgate"]
        self.assertGreater(r["site_min_radius_full_fuel_nm"], 30.0)
        self.assertAlmostEqual(r["site_fuel_max_kg"], hot["site_limit_tdp12_kg"] - 2068.5, delta=1.0)
        sl = next(c for c in d["conditions"] if c["condition"] == "nível do mar, ISA")
        self.assertEqual(sl["profiles"]["resgate"]["site_min_radius_full_fuel_nm"], 0.0)


class TestParallelPrediction(unittest.TestCase):
    def test_parallel_equals_sequential(self):
        from src.physics.helicopter.scenarios import cat_a_run, rescue_site_config
        cfg = rescue_site_config(1500.0, 25.0, 0.0)
        par = cat_a_run(2600.7, cfg=cfg, fail_rel_tdp_m=0.5, advisory=True)["advisory"]
        seq = cat_a_run(2600.7, cfg=cfg, fail_rel_tdp_m=0.5, advisory=True, advisory_parallel=False)["advisory"]
        self.assertIn(par["mode"], ("paralelo", "sequencial"))
        self.assertEqual(seq["mode"], "sequencial")
        for k in ("abortar", "prosseguir"):
            self.assertEqual(par["predicted"][k]["safe"], seq["predicted"][k]["safe"])
            self.assertEqual(par["predicted"][k]["reason"], seq["predicted"][k]["reason"])
            self.assertAlmostEqual(par["predicted"][k]["predicted_s"], seq["predicted"][k]["predicted_s"], places=6)


if __name__ == "__main__":
    unittest.main()
