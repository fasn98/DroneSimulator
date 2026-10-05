"""
Correction of the aval do Passo 4: which ground contact is the touchdown of a branch.

Rules (procedures.find_touchdown), checked here on constructed telemetry:
  (i)   the touchdown comes after the engine failure;
  (ii)  it comes after the aircraft has been airborne for at least 1 s (a skid bounce at lift-off is not it);
  (iii) it is the landing that ends the flight: the last contact before the stop, together with the bounces just
        before it; the sink rate judged is the largest among those contacts.
The constructed case reproduces the old bug: the old rule (first contact after the first lift-off) returns the
lift-off bounce at 0,2 m/s instead of the 4 m/s touchdown.
"""

import math
import unittest

import numpy as np

from src.physics.helicopter.procedures import evaluate_cat_a, evaluate_landing, find_touchdown

DT = 0.1


def profile(events, t_end=25.0):
    """Telemetry arrays from a list of (t0, t1, on_ground, vz, agl_skid) segments."""
    t = np.round(np.arange(0.0, t_end + 1e-9, DT), 3)
    g = np.zeros_like(t, bool)
    vz = np.zeros_like(t)
    h = np.zeros_like(t)
    for t0, t1, on, v, z in events:
        m = (t >= t0 - 1e-9) & (t < t1 - 1e-9)
        g[m], vz[m], h[m] = on, v, z
    return t, g, vz, h


# the Category A reject that exposed the bug: lift-off bounce at 0,3 s, flight, failure at 9,7 s, touchdown back
# on the deck at 16,3 s at ~4 m/s, a 1 s bounce, final contact at 17,5 s at 0,6 m/s, stopped on the deck
REJECT = [
    (0.0, 0.2, True, 0.0, 0.0),
    (0.2, 0.3, False, 0.27, 0.02),
    (0.3, 0.4, True, -0.2, 0.0),     # lift-off skid bounce, ~0,2 m/s
    (0.4, 3.8, True, 0.0, 0.0),
    (3.8, 16.3, False, 1.0, 8.0),
    (16.1, 16.3, False, -3.97, 0.3),  # descent onto the deck
    (16.3, 16.5, True, -3.27, 0.0),   # touchdown at ~4 m/s
    (16.5, 17.3, False, 0.55, 0.3),   # bounce, 1 s in the air, 0,3 m high
    (17.3, 17.5, False, -0.57, 0.1),
    (17.5, 25.01, True, -0.5, 0.0),   # final contact, 0,6 m/s, then stopped
]
FAIL_T = 9.7


def old_rule(t, g, vz):
    """The rule used until the aval do Passo 4: first contact after the first lift-off."""
    air = np.where(~g)[0]
    k = np.where(g & (np.arange(len(t)) > air[0]))[0][0]
    return float(t[k]), float(-min(vz[max(k - 2, 0):k + 1]))


class TestFindTouchdown(unittest.TestCase):
    def test_old_bug_reproduced_and_fixed(self):
        t, g, vz, h = profile(REJECT)
        t_old, sink_old = old_rule(t, g, vz)
        self.assertAlmostEqual(t_old, 0.3, places=6)  # the old rule picks the lift-off bounce ...
        self.assertLess(sink_old, 0.5)                 # ... at ~0,2 m/s: "safe"
        td = find_touchdown(t, g, vz, h, t_from=FAIL_T)
        self.assertAlmostEqual(td["t"], 16.3, places=6)
        self.assertAlmostEqual(td["sink_ms"], 3.97, places=2)  # the real touchdown: unsafe (> 1,5 m/s)
        self.assertEqual(td["n_bounces"], 1)
        self.assertAlmostEqual(td["t_last"], 17.5, places=6)

    def test_i_after_the_failure(self):
        # a contact before the failure (a touch-and-go at 2 s, after 1 s in the air) is not the touchdown
        ev = [(0.0, 0.5, True, 0.0, 0.0), (0.5, 2.0, False, 0.5, 1.0), (2.0, 2.3, True, -0.3, 0.0),
              (2.3, 12.0, False, 1.0, 10.0), (11.8, 12.0, False, -1.0, 0.2), (12.0, 25.01, True, -1.0, 0.0)]
        t, g, vz, h = profile(ev)
        self.assertAlmostEqual(find_touchdown(t, g, vz, h, t_from=None, min_air_s=1.0)["t"], 12.0, places=6)
        td = find_touchdown(t, g, vz, h, t_from=5.0)
        self.assertGreaterEqual(td["t"], 5.0)
        self.assertAlmostEqual(td["t"], 12.0, places=6)
        # failure after the last contact: no touchdown to judge
        self.assertIsNone(find_touchdown(t, g, vz, h, t_from=20.0))

    def test_ii_after_one_second_in_the_air(self):
        # only a lift-off bounce (0,1 s in the air) and nothing else: no touchdown
        ev = [(0.0, 0.2, True, 0.0, 0.0), (0.2, 0.3, False, 0.3, 0.02), (0.3, 25.01, True, -0.2, 0.0)]
        t, g, vz, h = profile(ev)
        self.assertIsNone(find_touchdown(t, g, vz, h, t_from=None, min_air_s=1.0))
        t, g, vz, h = profile(REJECT)
        td = find_touchdown(t, g, vz, h, t_from=None)  # even without the failure time, not the lift-off bounce
        self.assertAlmostEqual(td["t"], 16.3, places=6)
        k = td["k"]
        self.assertFalse(g[k - int(round(1.0 / DT)):k].any())  # airborne for the whole second before it

    def test_iii_last_contact_before_the_stop(self):
        # a firm contact, then a real flight again (3 s, 6 m high), then the final landing: the final one counts
        ev = [(0.0, 0.5, True, 0.0, 0.0), (0.5, 10.0, False, 1.0, 10.0), (9.8, 10.0, False, -2.5, 0.2),
              (10.0, 10.2, True, -2.5, 0.0), (10.2, 13.2, False, 1.0, 6.0), (13.0, 13.2, False, -0.8, 0.2),
              (13.2, 25.01, True, -0.8, 0.0)]
        t, g, vz, h = profile(ev)
        td = find_touchdown(t, g, vz, h, t_from=None)
        self.assertAlmostEqual(td["t"], 13.2, places=6)
        self.assertEqual(td["contacts"][-1], len(t) - 1 - int(round((25.0 - 13.2) / DT)))
        self.assertTrue(g[td["k_last"]:].all())  # on the ground from the last contact to the stop
        self.assertAlmostEqual(td["sink_ms"], 0.8, places=6)
        # with bounces only (REJECT), the landing groups them and the largest sink counts
        t, g, vz, h = profile(REJECT)
        td = find_touchdown(t, g, vz, h, t_from=FAIL_T)
        self.assertTrue(g[td["k_last"]:].all())
        self.assertGreater(td["sink_ms"], 3.0)


class _Tel:
    def __init__(self, cols):
        self.cols = cols

    def column(self, k):
        return np.asarray(self.cols[k], float)


class _Pad:
    pad_x = pad_y = 0.0
    pad_half_size = 10.0

    def on_pad(self, x, y, margin=0.0):
        return abs(x) <= self.pad_half_size - margin and abs(y) <= self.pad_half_size - margin


class _Sim:
    def __init__(self):
        self.fail_time = FAIL_T
        self.p = type("P", (), {"cg_h": 1.3})()
        self.dyn = type("D", (), {"terrain": _Pad()})()


class TestEvaluatorsUseTheRule(unittest.TestCase):
    def setUp(self):
        t, g, vz, h = profile(REJECT)
        n = len(t)
        z = h + 1.3
        self.tel = _Tel({"t": t, "on_ground": g.astype(float), "vz": vz, "x": np.zeros(n), "y": np.zeros(n),
                         "z": z, "agl": z, "vx": np.zeros(n), "vy": np.zeros(n), "roll_deg": np.zeros(n),
                         "pitch_deg": np.zeros(n), "nr_pct": np.full(n, 100.0), "tas_ms": np.zeros(n)})

    def test_reject_branch(self):
        ev = evaluate_cat_a(self.tel, _Sim(), 25.0, "reject")
        self.assertAlmostEqual(ev["touchdown_t"], 16.3, places=6)
        self.assertGreater(ev["touchdown_sink_ms"], 1.5)
        self.assertFalse(ev["safe"])
        self.assertIn("toque a 4,0 m/s".replace(",", "."), ev["reason"])

    def test_landing(self):
        L = evaluate_landing(self.tel, t_from=FAIL_T)
        self.assertTrue(L["landed"])
        self.assertAlmostEqual(L["touchdown_t"], 16.3, places=6)
        self.assertEqual(L["outcome"]["class"], "dano_provavel")
        self.assertTrue(math.isclose(L["touchdown_sink_ms"], 3.97, abs_tol=0.01))


if __name__ == "__main__":
    unittest.main()
