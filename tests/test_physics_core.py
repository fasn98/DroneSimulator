"""
Verification tests for the Twin v2 physics core.

Run with either:
    python -m pytest tests/ -v
    python -m unittest discover -s tests -v
"""

import math
import unittest

import numpy as np

from src.physics import (Body, RigidBodyDynamics, TwinSimulator, hover_at, hover_report, list_vehicles,
                         load_body, load_vehicle, max_hover_mass, quat_from_euler)
from src.physics.atmosphere import G0

MSH_BODY = Body.with_surface_density(load_body("mars"), density=0.015, temperature=223.15)


class TestAtmosphere(unittest.TestCase):
    def test_earth_sea_level(self):
        earth = load_body("earth")
        self.assertAlmostEqual(earth.density(0.0), 1.225, delta=0.005)
        self.assertAlmostEqual(earth.speed_of_sound(0.0), 340.3, delta=1.0)

    def test_mars_is_thin_co2(self):
        mars = load_body("mars")
        self.assertTrue(0.010 < mars.density(0.0) < 0.025)
        self.assertLess(mars.speed_of_sound(0.0), 245.0)  # CO2 at ~210 K

    def test_moon_is_vacuum(self):
        moon = load_body("moon")
        self.assertEqual(moon.density(0.0), 0.0)
        self.assertEqual(moon.pressure(100.0), 0.0)

    def test_msh_design_point(self):
        self.assertAlmostEqual(MSH_BODY.density(0.0), 0.015, places=6)
        # NASA MSH: M_tip 0.8 -> 186.5 m/s tip speed at -50 C
        self.assertAlmostEqual(0.8 * MSH_BODY.speed_of_sound(0.0), 186.5, delta=1.0)


class TestRotor(unittest.TestCase):
    def test_rotor_makes_no_thrust_in_vacuum(self):
        moon = load_body("moon")
        v = load_vehicle("mars_hexacopter_tcc")
        self.assertEqual(v.max_vertical_force(moon), 0.0)

    def test_thrust_scales_with_density(self):
        v = load_vehicle("default_quadrotor")
        earth, mars = load_body("earth"), load_body("mars")
        ratio = v.max_vertical_force(mars) / v.max_vertical_force(earth)
        # T_max ~ rho * V_tip^2; on Mars V_tip is also capped lower by the slower speed of sound
        r = v.rotors[0]
        vt_e = r.tip_speed(earth.density(0), earth.speed_of_sound(0))
        vt_m = r.tip_speed(mars.density(0), mars.speed_of_sound(0))
        self.assertLess(vt_m, vt_e)
        expected = (mars.density(0) / earth.density(0)) * (vt_m / vt_e) ** 2
        self.assertAlmostEqual(ratio, expected, delta=1e-9)
        self.assertLess(ratio, 0.02)  # a drone sized for Earth keeps < 2% of its thrust on Mars

    def test_msh_hover_power_reproduced(self):
        """Implementation check: NASA's own parameters must give NASA's 6.2 kW hover power."""
        r = hover_report(load_vehicle("mars_hexacopter_tcc"), MSH_BODY)
        self.assertTrue(r.can_hover)
        self.assertLess(abs(r.hover_power_w - 6200.0) / 6200.0, 0.15)
        self.assertAlmostEqual(r.ct_sigma_hover, 0.115, delta=0.005)


class TestThesisCorrection(unittest.TestCase):
    def test_tcc_70kg_mars_drone_cannot_hover(self):
        v = load_vehicle("tcc_mars_70kg_original")
        self.assertLess(v.thrust_to_weight(MSH_BODY), 1.0)
        self.assertLess(max_hover_mass(v, MSH_BODY), 10.0)  # ~9 kg even with MSH-grade rotors

    def test_tcc_70kg_stays_on_ground_in_simulation(self):
        v = load_vehicle("tcc_mars_70kg_original")
        sim = TwinSimulator(v, load_body("mars"), seed=0)
        tel = sim.run(8.0, hover_at([0, 0, 10]))
        self.assertLess(tel.column("z").max(), v.gear_height + 0.01)

    def test_rotorcraft_cannot_fly_on_the_moon(self):
        v = load_vehicle("mars_hexacopter_tcc")
        sim = TwinSimulator(v, load_body("moon"), seed=0)
        tel = sim.run(5.0, hover_at([0, 0, 10]))
        self.assertLess(tel.column("z").max(), v.gear_height + 0.01)

    def test_validator_flags_infeasible_legacy_configs(self):
        earth = load_body("earth")
        for name in ("professional_quadcopter_optimized", "acoustic_hexacopter_optimized",
                     "precision_octocopter_optimized"):
            warnings = load_vehicle(name).validate(earth)
            self.assertTrue(any("cannot lift off" in w for w in warnings), name)

    def test_every_config_loads(self):
        for name in list_vehicles():
            v = load_vehicle(name)
            self.assertGreater(v.gross_mass, 0.0)
            self.assertGreater(v.actuator_count, 0)


class TestRigidBody(unittest.TestCase):
    def _free_fall(self, body_name):
        body = load_body(body_name)
        v = load_vehicle("lunar_hopper_tcc")
        v.drag_area = 0.0
        dyn = RigidBodyDynamics(v, body)
        x = dyn.initial_state(position=(0.0, 0.0, 200.0))
        cmd = np.zeros(dyn.n_act)
        for _ in range(400):  # 2 s at 5 ms
            x = dyn.rk4_step(x, cmd, 0.005)
        return x, body

    def test_free_fall_matches_gravity_on_every_body(self):
        for name in ("earth", "mars", "moon"):
            x, body = self._free_fall(name)
            expected = 200.0 - 0.5 * body.gravity * 2.0 ** 2
            self.assertAlmostEqual(x[2], expected, delta=1e-6, msg=name)

    def test_torque_free_tumble_conserves_energy_and_momentum(self):
        body = load_body("moon")
        v = load_vehicle("lunar_hopper_tcc")
        v.inertia = np.diag([5.0, 8.0, 11.0])  # asymmetric -> non-trivial tumbling
        dyn = RigidBodyDynamics(v, body)
        x = dyn.initial_state(position=(0.0, 0.0, 1000.0))
        x[6:10] = quat_from_euler(0.3, -0.2, 1.0)
        x[10:13] = [0.4, 1.5, -0.3]
        I = v.inertia

        def invariants(s):
            from src.physics.dynamics import quat_to_rot
            w = s[10:13]
            return 0.5 * w @ I @ w, quat_to_rot(s[6:10]) @ (I @ w)

        e0, h0 = invariants(x)
        cmd = np.zeros(dyn.n_act)
        for _ in range(4000):  # 20 s
            x = dyn.rk4_step(x, cmd, 0.005)
            self.assertAlmostEqual(np.linalg.norm(x[6:10]), 1.0, delta=1e-9)
        e1, h1 = invariants(x)
        self.assertLess(abs(e1 - e0) / e0, 1e-6)
        self.assertLess(np.linalg.norm(h1 - h0) / np.linalg.norm(h0), 1e-6)


class TestClosedLoop(unittest.TestCase):
    def test_mars_hexacopter_holds_hover_in_wind(self):
        v = load_vehicle("mars_hexacopter_tcc")
        body = load_body("mars")
        sim = TwinSimulator(v, body, seed=3)
        tel = sim.run(40.0, hover_at([0, 0, 10]))
        late = tel.column("t") > 20.0
        z, x, y = tel.column("z")[late], tel.column("x")[late], tel.column("y")[late]
        self.assertLess(np.abs(z - 10.0).max(), 0.5)
        self.assertLess(np.hypot(x, y).max(), 0.5)
        # time-domain power agrees with the closed-form hover estimate
        p_sim = tel.column("power_w")[late].mean()
        p_ref = hover_report(v, body, altitude=10.0).hover_power_w
        self.assertLess(abs(p_sim - p_ref) / p_ref, 0.05)

    def test_battery_energy_equals_integrated_power(self):
        v = load_vehicle("mars_hexacopter_tcc")
        sim = TwinSimulator(v, load_body("mars"), seed=0, wind=False)
        tel = sim.run(20.0, hover_at([0, 0, 5]), record_every=0.005)
        used = v.battery_wh - tel.column("battery_wh")[-1]
        integ = np.trapezoid(tel.column("power_w"), tel.column("t")) / 3600.0
        self.assertLess(abs(used - integ) / integ, 0.02)

    def test_lunar_hopper_propellant_follows_rocket_equation(self):
        v = load_vehicle("lunar_hopper_tcc")
        moon = load_body("moon")
        sim = TwinSimulator(v, moon, seed=0)
        tel = sim.run(30.0, hover_at([0, 0, 10]), record_every=0.005)
        late = tel.column("t") > 15.0
        self.assertLess(np.abs(tel.column("z")[late] - 10.0).max(), 0.5)
        used = v.propellant_mass - tel.column("propellant_kg")[-1]
        expected = np.trapezoid(tel.column("thrust_total_n"), tel.column("t")) / (230.0 * G0)
        self.assertLess(abs(used - expected) / expected, 0.03)

    def test_legacy_earth_quadrotor_still_flies(self):
        sim = TwinSimulator(load_vehicle("default_quadrotor"), load_body("earth"), seed=0)
        tel = sim.run(20.0, hover_at([0, 0, 5]))
        self.assertLess(abs(tel.column("z")[-20:].mean() - 5.0), 0.3)


if __name__ == "__main__":
    unittest.main()
