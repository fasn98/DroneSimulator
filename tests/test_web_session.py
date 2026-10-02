"""
Tests for the web-app session (src/physics/web_session.py): the layer that lets
simulation_server.py fly missions with the Twin v2 physics core.

Run with either:
    python -m pytest tests/ -v
    python -m unittest discover -s tests -t . -v
"""

import json
import math
import unittest
from pathlib import Path

import numpy as np

from src.physics import body_from_config, waypoint_route
from src.physics.web_session import LEGACY_KEYS, PHYSICS_KEYS, WebSession

MISSIONS = json.loads((Path(__file__).resolve().parents[1] / "config" / "missions.json").read_text())
RECON = MISSIONS["reconhecimento_marte_tcc"]


def fly(session, record_every=1.0):
    """Run a session to the end, returning telemetry snapshots taken every `record_every` s."""
    rows = [session.telemetry()]
    while not session.finished:
        session.advance(record_every)
        rows.append(session.telemetry())
    return rows


def assert_plain_json(test, obj, path="tel"):
    """Only JSON-native Python types, and no inf / NaN anywhere."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            test.assertIsInstance(k, str)
            assert_plain_json(test, v, f"{path}.{k}")
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            assert_plain_json(test, v, f"{path}[{i}]")
    else:
        test.assertIn(type(obj), (str, int, float, bool, type(None)), f"{path} is {type(obj).__name__}")
        if type(obj) is float:
            test.assertTrue(math.isfinite(obj), f"{path} = {obj}")


class TestMarsHexacopterMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = WebSession("mars_hexacopter_tcc", RECON, "mars", seed=3)
        cls.rows = fly(cls.s)

    def test_completes_all_waypoints(self):
        s = self.s
        self.assertEqual(s.status, "completed", s.failure_reason)
        self.assertEqual(s.progress, 1.0)
        self.assertEqual(s.current_waypoint, len(RECON["waypoints"]))
        self.assertEqual(self.rows[-1]["mission_progress"], 100.0)
        self.assertEqual(self.rows[-1]["mission_status"], "completed")
        self.assertLess(s.t, 300.0)

    def test_flies_the_route_and_lands(self):
        s, last = self.s, self.rows[-1]
        self.assertGreater(s.max_height, 14.0)
        self.assertLess(s.max_height, 17.0)
        self.assertTrue(last["on_ground"])
        self.assertLess(math.hypot(last["position"]["x"], last["position"]["y"]), 1.5)
        # Ideal path: 15 up + 40 + 30 + 40 + 30 + 15 down = 170 m; no large overshoot loops
        self.assertGreater(s.total_distance, 165.0)
        self.assertLess(s.total_distance, 200.0)

    def test_progress_is_monotonic(self):
        p = [r["mission_progress"] for r in self.rows]
        self.assertTrue(all(b >= a for a, b in zip(p, p[1:])))

    def test_energy_and_hover_power(self):
        used = self.s.vehicle.battery_wh - self.rows[-1]["battery_wh"]
        self.assertGreater(used, 50.0)
        self.assertLess(used, self.s.vehicle.battery_wh)
        airborne = [r["power_w"] for r in self.rows if not r["on_ground"]]
        # Hover power of the MSH-class vehicle at this density is ~6 kW (closed form: hover_report)
        self.assertAlmostEqual(float(np.median(airborne)), self.s.report.hover_power_w, delta=0.15 * self.s.report.hover_power_w)

    def test_attitude_from_quaternion_in_radians(self):
        pitches = [abs(r["attitude"]["pitch"]) for r in self.rows]
        rolls = [abs(r["attitude"]["roll"]) for r in self.rows]
        self.assertGreater(max(pitches + rolls), math.radians(1.0))  # it really tilts to fly
        self.assertLess(max(pitches + rolls), math.radians(30.0))  # within the 25 deg tilt limit + gusts

    def test_thin_air_telemetry(self):
        mid = self.rows[len(self.rows) // 2]
        self.assertTrue(0.010 < mid["air_density"] < 0.020)
        self.assertAlmostEqual(mid["tip_mach"], 0.8, places=3)
        self.assertGreater(mid["thrust_to_weight"], 1.0)
        self.assertEqual(mid["propellant_kg"], 0.0)


class TestInfeasibleVehicle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = WebSession("tcc_mars_70kg_original", RECON, "mars", seed=3)
        cls.rows = fly(cls.s)

    def test_warning_before_flight(self):
        f = self.s.feasibility()
        self.assertFalse(f["can_fly"])
        self.assertEqual(f["warning_level"], "danger")
        self.assertIn("não consegue decolar em Marte", f["warning"])
        self.assertRegex(f["warning"], r"T/W = 0,1\d")
        self.assertLess(f["thrust_to_weight"], 0.2)
        self.assertIsNone(f["hover_power_w"])  # closed form gives inf -> null in JSON
        json.dumps(f, allow_nan=False)

    def test_never_climbs_and_fails(self):
        s = self.s
        self.assertLess(s.max_height, s.rest_height + 0.05)
        self.assertTrue(all(r["on_ground"] for r in self.rows))
        self.assertEqual(s.status, "failed")
        self.assertEqual(s.failure_reason, s.warning_pt)
        self.assertAlmostEqual(s.t, s.ground_timeout, delta=0.01)
        self.assertEqual(self.rows[-1]["mission_progress"], 0.0)


class TestLunarHopperMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = WebSession("lunar_hopper_tcc", RECON, "moon", seed=3)
        cls.rows = fly(cls.s)

    def test_completes_without_warning(self):
        self.assertIsNone(self.s.feasibility()["warning"])
        self.assertEqual(self.s.status, "completed", self.s.failure_reason)
        self.assertTrue(self.rows[-1]["on_ground"])

    def test_consumes_propellant(self):
        p0, p1 = self.rows[0]["propellant_kg"], self.rows[-1]["propellant_kg"]
        self.assertAlmostEqual(p0, 35.0)
        self.assertGreater(p0 - p1, 2.0)
        self.assertLess(p0 - p1, 15.0)
        props = [r["propellant_kg"] for r in self.rows]
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(props, props[1:])))

    def test_vacuum_telemetry(self):
        mid = self.rows[len(self.rows) // 2]
        self.assertEqual(mid["air_density"], 0.0)
        self.assertEqual(mid["tip_mach"], 0.0)
        self.assertEqual(mid["wind_speed"], 0.0)
        self.assertGreater(mid["thrust_to_weight"], 1.0)


class TestRotorcraftOnTheMoon(unittest.TestCase):
    def test_hexacopter_gets_warning_and_stays_down(self):
        s = WebSession("mars_hexacopter_tcc", RECON, "moon", seed=3)
        f = s.feasibility()
        self.assertFalse(f["can_fly"])
        self.assertIn("não consegue decolar na Lua", f["warning"])
        self.assertIn("T/W = 0,00", f["warning"])
        fly(s)
        self.assertEqual(s.status, "failed")
        self.assertLess(s.max_height, s.rest_height + 0.05)


class TestTelemetryContract(unittest.TestCase):
    def test_keys_and_json(self):
        for vehicle, env in [("mars_hexacopter_tcc", "mars"), ("lunar_hopper_tcc", "moon"),
                             ("tcc_mars_70kg_original", "mars"), ("mars_hexacopter_tcc", "moon")]:
            with self.subTest(vehicle=vehicle, env=env):
                s = WebSession(vehicle, RECON, env, seed=1)
                s.advance(12.0)  # mid-climb for the flyers
                for obj in (s.telemetry(), s.environment_data(), s.feasibility()):
                    assert_plain_json(self, obj)
                    json.dumps(obj, allow_nan=False)
                tel = s.telemetry()
                for key in LEGACY_KEYS + PHYSICS_KEYS:
                    self.assertIn(key, tel)
                self.assertEqual(set(tel["position"]), {"x", "y", "z"})
                self.assertEqual(set(tel["velocity"]), {"x", "y", "z"})
                self.assertEqual(set(tel["attitude"]), {"roll", "pitch", "yaw"})
                self.assertIsInstance(tel["on_ground"], bool)
                self.assertIsInstance(tel["current_waypoint"], int)
                self.assertAlmostEqual(tel["altitude"], tel["position"]["z"])
                self.assertAlmostEqual(tel["ground_speed"], math.hypot(tel["velocity"]["x"], tel["velocity"]["y"]),
                                       places=3)
                self.assertTrue(0.0 <= tel["battery_pct"] <= 100.0)


class TestEnvironmentAndRoute(unittest.TestCase):
    def test_custom_environment_uses_typed_density(self):
        # The web UI sends density + temperature with a fixed 101325 Pa; density must win.
        cfg = {"gravity": 3.71, "atmosphere": {"sea_level_density": 0.02, "sea_level_temperature": 220.0,
                                               "sea_level_pressure": 101325.0, "gas_constant": 287.0,
                                               "temperature_lapse_rate": -0.0065},
               "wind": {"enabled": True, "base_speed": 4.0, "direction": 90.0, "gust_factor": 1.5}}
        b = body_from_config("custom", cfg, prefer_density=True)
        self.assertAlmostEqual(b.density(0.0), 0.02, places=6)
        self.assertEqual(b.wind_direction_deg, 90.0)
        cfg["atmosphere"]["sea_level_density"] = 0.0
        self.assertFalse(body_from_config("custom", cfg, prefer_density=True).has_atmosphere)

    def test_custom_environment_session(self):
        cfg = {"gravity": 9.81, "atmosphere": {"sea_level_density": 1.225, "sea_level_temperature": 288.15,
                                               "sea_level_pressure": 101325.0, "gas_constant": 287.0}}
        s = WebSession("default_quadrotor", RECON, "custom", environment_config=cfg, seed=1)
        self.assertTrue(s.can_fly)
        s.advance(5.0)
        self.assertGreater(s.max_height, s.rest_height + 1.0)

    def test_route_per_waypoint_values_and_accel_limit(self):
        wps = [[0, 0, 5], [20, 0, 5]]
        route = waypoint_route(wps, cruise_speed=4.0, climb_rate=1.0, hold_time=[0.5, 1.0],
                               acceptance=[0.5, 0.5], max_accel=0.5)
        x = np.zeros(13)
        speeds, t, dt = [], 0.0, 0.02
        for _ in range(5000):  # perfect tracker: the vehicle sits on the reference
            ref, vel, _ = route(t, x)
            x[0:3] = ref
            speeds.append(np.linalg.norm(vel))
            t += dt
        self.assertEqual(route.progress(), 1.0)
        accel = np.abs(np.diff(speeds)) / dt
        # Within 10% of the limit (the discrete sqrt(2 a d) braking curve overshoots it by O(dt))
        self.assertLess(np.percentile(accel, 99), 0.55)
        self.assertLessEqual(max(speeds), 4.0 + 1e-9)
        with self.assertRaises(ValueError):
            waypoint_route(wps, hold_time=[1.0])


if __name__ == "__main__":
    unittest.main()
