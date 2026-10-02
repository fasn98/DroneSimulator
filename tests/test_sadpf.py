"""
Phase 3 verification: sensors, EKF navigation and the SADPF (fault detection,
isolation and autonomous actions). Vehicle: TCC Mars hexacopter on Mars.
"""

import dataclasses
import json
import unittest

import numpy as np

from src.physics import (RotorFault, SensorConfig, SensorFault, SensorSuite, TwinSimulator, hover_at, load_body,
                         load_vehicle)
from src.physics.web_session import WebSession

# Operating-envelope wind for a Mars rotorcraft (Ingenuity was cleared for about 10 m/s); the
# app's default Mars environment (15 +- 15 m/s) is dust-storm level, see docs/twin_v3.
MARS = dataclasses.replace(load_body("mars"), wind_mean=5.0, wind_gust_std=2.0)
HEXA = load_vehicle("mars_hexacopter_tcc")


def fly(faults=(), duration=40.0, seed=2, **kw):
    sim = TwinSimulator(HEXA, MARS, seed=seed, sensors=SensorConfig(), sadpf=True, faults=list(faults), **kw)
    tel = sim.run(duration, hover_at([0.0, 0.0, 10.0]))
    return sim, tel


def diag_events(sim):
    return [e for e in sim.sadpf.events if e.code not in ("landed", "ekf_reset", "gyro_bias")]


def max_tilt_deg(tel):
    return float(np.max(np.hypot(tel.column("roll_deg"), tel.column("pitch_deg"))))


class TestSensors(unittest.TestCase):
    def test_baro_altitude_noise_follows_air_density(self):
        rng = np.random.default_rng(0)
        mars = SensorSuite(MARS, rng).baro_altitude_noise(0.0)
        earth = SensorSuite(load_body("earth"), rng).baro_altitude_noise(0.0)
        self.assertGreater(mars, 2.0)  # ~3.5 m: why Mars vehicles do not navigate on the barometer
        self.assertLess(earth, 0.05)
        self.assertEqual(SensorSuite(load_body("moon"), rng).baro_altitude_noise(0.0), float("inf"))


class TestNominal(unittest.TestCase):
    def test_flies_on_estimate_without_false_alarms(self):
        sim, tel = fly(duration=40.0)
        t = tel.column("t")
        hover = t > 15
        self.assertLess(np.max(tel.column("pos_err_m")[hover]), 0.6)
        self.assertLess(np.max(np.abs(tel.column("z")[hover] - 10.0)), 0.6)
        self.assertEqual(diag_events(sim), [])
        self.assertEqual(sim.sadpf.level, 0)


class TestActuatorFaults(unittest.TestCase):
    def test_rotor_loss_of_effectiveness_is_isolated_and_compensated(self):
        sim, tel = fly([RotorFault(2, 0.7, 20.0)])
        ev = diag_events(sim)
        self.assertEqual(len(ev), 1, [e.message for e in ev])
        e = ev[0]
        self.assertEqual(e.code, "rotor_loe")
        self.assertEqual(e.data["rotor"], 2)
        self.assertLess(e.t - 20.0, 2.0)
        self.assertAlmostEqual(e.data["loe"], 0.3, delta=0.12)
        self.assertEqual(e.level, 2)  # 30 % LOE leaves T/W ~1.13: the mission continues
        self.assertGreater(e.data["tw_post"], 1.1)
        self.assertLess(sim.ctrl.alloc.effectiveness[2], 0.85)  # allocation reconfigured
        self.assertIsNone(sim.sadpf.land_requested)
        self.assertLess(abs(tel.column("z")[-1] - 10.0), 1.0)

    def test_post_fault_margin_decides_landing(self):
        nominal = np.ones(6)
        half = nominal.copy()
        half[2] = 0.5
        sim, tel = fly([RotorFault(2, 0.5, 20.0)], duration=50.0)
        s = sim.sadpf
        self.assertAlmostEqual(s.post_fault_thrust_to_weight(nominal, 0.0, HEXA.gross_mass),
                               HEXA.thrust_to_weight(MARS), delta=1e-6)
        self.assertLess(s.post_fault_thrust_to_weight(half, 0.0, HEXA.gross_mass), 1.1)
        ev = diag_events(sim)
        self.assertEqual(ev[0].level, 3)  # can still hover, but not manoeuvre: land
        self.assertTrue(s.landed)
        self.assertLess(max_tilt_deg(tel), 60.0)

    def test_rotor_failure_triggers_emergency_landing(self):
        sim, tel = fly([RotorFault(4, 0.0, 20.0)], duration=40.0)
        ev = diag_events(sim)
        self.assertTrue(ev and ev[0].code == "rotor_loe" and ev[0].data["rotor"] == 4)
        self.assertEqual(ev[0].level, 3)
        self.assertTrue(sim.sadpf.landed)
        self.assertLess(max_tilt_deg(tel), 60.0)  # lands upright
        self.assertLess(tel.column("z")[-1], HEXA.gear_height + 0.1)


class TestSensorFaults(unittest.TestCase):
    def test_stuck_altimeter_is_isolated(self):
        sim, tel = fly([SensorFault("altimeter", "stuck", 20.0)], duration=35.0)
        self.assertIn("altimeter", sim.sadpf.sensors.isolated)
        self.assertEqual(sim.sadpf.sensors.isolated["altimeter"], "stuck")
        self.assertLess(abs(tel.column("z")[-1] - 10.0), 1.0)

    def test_gyro_fault_in_one_imu_is_outvoted(self):
        sim, tel = fly([SensorFault("gyro", "bias", 20.0, np.array([0.0, 0.06, 0.0]), unit=1)], duration=35.0)
        self.assertEqual(sim.sadpf.sensors.isolated.get("gyro#1"), "vote")
        self.assertLess(np.max(tel.column("pos_err_m")[tel.column("t") > 20]), 0.8)
        self.assertLess(abs(tel.column("z")[-1] - 10.0), 1.0)

    def test_lost_position_reference_lands(self):
        sim, tel = fly([SensorFault("nav_pos", "bias", 20.0, np.array([4.0, 0.0, 0.0]))], duration=45.0)
        self.assertIn("nav_pos", sim.sadpf.sensors.isolated)
        self.assertEqual(sim.sadpf.level, 3)
        self.assertIsNotNone(sim.sadpf.land_requested)
        self.assertLess(max_tilt_deg(tel), 60.0)


class TestWebSession(unittest.TestCase):
    def test_fault_injection_and_telemetry(self):
        mission = {"waypoints": [{"x": 0, "y": 0, "z": 10, "tolerance": 2.0, "duration": 30.0}]}
        ws = WebSession("mars_hexacopter_tcc", mission, environment="mars", seed=4, sadpf=True)
        ws.advance(18.0)
        msg = ws.inject_fault({"type": "rotor", "rotor": 2, "effectiveness": 0.45})
        self.assertIn("Rotor 2", msg)
        ws.advance(5.0)
        tel = ws.telemetry()
        json.dumps(tel, allow_nan=False)  # strict JSON for the browser
        s = tel["sadpf"]
        self.assertGreaterEqual(s["level"], 2)
        self.assertTrue(any("Rotor 2" in e["message"] for e in s["events"]))
        self.assertLess(s["rotor_effectiveness"][1], 0.75)
        with self.assertRaises(ValueError):
            ws.inject_fault({"type": "rotor", "rotor": 9})

    def test_sadpf_off_keeps_phase2_telemetry(self):
        ws = WebSession("mars_hexacopter_tcc", {}, environment="mars", seed=1)
        ws.advance(1.0)
        self.assertIsNone(ws.telemetry()["sadpf"])
        with self.assertRaises(ValueError):
            ws.inject_fault({"type": "rotor", "rotor": 1})


if __name__ == "__main__":
    unittest.main()
