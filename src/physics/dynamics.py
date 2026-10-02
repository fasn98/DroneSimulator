"""
Rigid-body 6-DoF dynamics with a true 4th-order Runge-Kutta integrator.

Frames: world = ENU (x east, y north, z up, ground at z = 0); body = x forward,
y left, z up. Attitude is a unit quaternion q = [w, x, y, z] rotating body
vectors into the world frame, so there is no gimbal lock and no Euler-rate
approximation.

State vector layout (n = number of actuators):
    [0:3]   position, world (m)
    [3:6]   velocity, world (m/s)
    [6:10]  attitude quaternion
    [10:13] angular rate, body (rad/s)
    [13:13+n] actuator force, rotors first then thrusters (N)
    [13+n]  battery energy remaining (Wh)
    [14+n]  propellant remaining (kg)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from .atmosphere import Body
from .vehicle import Vehicle


# ---------------------------------------------------------------------- #
# Quaternion helpers
# ---------------------------------------------------------------------- #
def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_derivative(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    p, qq, r = omega_body
    return 0.5 * np.array([
        -x * p - y * qq - z * r,
        w * p + y * r - z * qq,
        w * qq - x * r + z * p,
        w * r + x * qq - y * p,
    ])


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def euler_from_quat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


# ---------------------------------------------------------------------- #
@dataclass
class Outputs:
    """Derived quantities at a given state (for telemetry, not integrated)."""
    density: float
    sound_speed: float
    electrical_power: float  # W, rotors + avionics + heaters
    rotor_power: float  # W, electrical, rotors only
    max_tip_mach: float
    on_ground: bool


class RigidBodyDynamics:
    GROUND_DEFLECTION = 0.005  # m of leg compression at static load
    GROUND_DAMPING_RATIO = 0.8
    GROUND_FRICTION = 0.6

    def __init__(self, vehicle: Vehicle, body: Body):
        self.vehicle = vehicle
        self.body = body
        self.n_rot = len(vehicle.rotors)
        self.n_thr = len(vehicle.thrusters)
        self.n_act = self.n_rot + self.n_thr
        self.inertia = vehicle.inertia
        self.inertia_inv = np.linalg.inv(vehicle.inertia)
        g_r, g_h = vehicle.gear_radius, vehicle.gear_height
        self.feet = [np.array([g_r, 0, -g_h]), np.array([-g_r, 0, -g_h]),
                     np.array([0, g_r, -g_h]), np.array([0, -g_r, -g_h])]
        self.tau = np.array([r.time_constant for r in vehicle.rotors] +
                            [t.time_constant for t in vehicle.thrusters])
        self.wind = np.zeros(3)

    # ------------------------------------------------------------------ #
    def initial_state(self, position=(0.0, 0.0, None), yaw: float = 0.0) -> np.ndarray:
        x = np.zeros(15 + self.n_act)
        pos = np.array(position, dtype=object)
        if pos[2] is None:  # resting on its legs
            pos[2] = self.vehicle.gear_height - self.GROUND_DEFLECTION
        x[0:3] = np.array(pos, dtype=float)
        x[6:10] = quat_from_euler(0.0, 0.0, yaw)
        x[13 + self.n_act] = self.vehicle.battery_wh
        x[14 + self.n_act] = self.vehicle.propellant_mass
        return x

    def mass(self, x: np.ndarray) -> float:
        return self.vehicle.mass_dry + self.vehicle.payload + max(x[14 + self.n_act], 0.0)

    def actuator_limits(self, x: np.ndarray) -> np.ndarray:
        """Current max force per actuator (depends on altitude, health and remaining energy)."""
        alt = x[2]
        rho, a = self.body.density(alt), self.body.speed_of_sound(alt)
        lim = np.array([r.max_thrust(rho, a) for r in self.vehicle.rotors] +
                       [t.health * t.max_force for t in self.vehicle.thrusters])
        if self.n_rot and x[13 + self.n_act] <= 0.0:
            lim[:self.n_rot] = 0.0
        if self.n_thr and x[14 + self.n_act] <= 0.0:
            lim[self.n_rot:] = 0.0
        return lim

    # ------------------------------------------------------------------ #
    def derivative(self, x: np.ndarray, cmd: np.ndarray) -> np.ndarray:
        v = self.vehicle
        n = self.n_act
        pos, vel, q, w = x[0:3], x[3:6], x[6:10], x[10:13]
        act = x[13:13 + n]
        R = quat_to_rot(q / np.linalg.norm(q))
        m = self.mass(x)

        rho = self.body.density(pos[2])
        a = self.body.speed_of_sound(pos[2])
        limits = self.actuator_limits(x)
        f_act = np.clip(act, 0.0, limits)

        force_b = np.zeros(3)
        moment_b = np.zeros(3)
        p_rotor = 0.0
        for i, r in enumerate(v.rotors):
            t = f_act[i]
            f = np.array([0.0, 0.0, t])
            force_b += f
            moment_b += np.cross(r.position, f)
            moment_b[2] += -r.spin * r.torque(t, rho, a)
            p_rotor += r.electrical_power(t, rho)
        mdot = 0.0
        for j, th in enumerate(v.thrusters):
            t = f_act[self.n_rot + j]
            f = th.direction * t
            force_b += f
            moment_b += np.cross(th.position, f)
            mdot += th.mass_flow(t)

        force_w = R @ force_b + np.array([0.0, 0.0, -m * self.body.gravity])

        # Aerodynamic drag on the airframe, relative to the moving air mass
        if rho > 0.0:
            v_rel = vel - self.wind
            force_w += -0.5 * rho * v.drag_area * np.linalg.norm(v_rel) * v_rel

        # Ground contact: spring-damper legs with Coulomb-capped viscous friction
        k = m * self.body.gravity / (len(self.feet) * self.GROUND_DEFLECTION)
        c = 2.0 * self.GROUND_DAMPING_RATIO * np.sqrt(k * m / len(self.feet))
        for foot in self.feet:
            pf = pos + R @ foot
            if pf[2] < 0.0:
                vf = vel + R @ np.cross(w, foot)
                fz = max(k * (-pf[2]) - c * vf[2], 0.0)
                ft = -c * vf[:2]
                cap = self.GROUND_FRICTION * fz
                nt = np.linalg.norm(ft)
                if nt > cap and nt > 0:
                    ft *= cap / nt
                f_ground = np.array([ft[0], ft[1], fz])
                force_w += f_ground
                moment_b += np.cross(foot, R.T @ f_ground)

        dx = np.zeros_like(x)
        dx[0:3] = vel
        dx[3:6] = force_w / m
        dx[6:10] = quat_derivative(q, w)
        dx[10:13] = self.inertia_inv @ (moment_b - np.cross(w, self.inertia @ w))
        dx[13:13 + n] = (np.clip(cmd, 0.0, limits) - act) / self.tau
        dx[13 + n] = -(p_rotor + v.avionics_w + v.heater_w) / 3600.0 if x[13 + n] > 0 else 0.0
        dx[14 + n] = -mdot if x[14 + n] > 0 else 0.0
        return dx

    def rk4_step(self, x: np.ndarray, cmd: np.ndarray, dt: float) -> np.ndarray:
        k1 = self.derivative(x, cmd)
        k2 = self.derivative(x + 0.5 * dt * k1, cmd)
        k3 = self.derivative(x + 0.5 * dt * k2, cmd)
        k4 = self.derivative(x + dt * k3, cmd)
        xn = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        xn[6:10] /= np.linalg.norm(xn[6:10])
        n = self.n_act
        xn[13 + n] = max(xn[13 + n], 0.0)
        xn[14 + n] = max(xn[14 + n], 0.0)
        return xn

    # ------------------------------------------------------------------ #
    def outputs(self, x: np.ndarray) -> Outputs:
        alt = x[2]
        rho, a = self.body.density(alt), self.body.speed_of_sound(alt)
        f = np.clip(x[13:13 + self.n_act], 0.0, self.actuator_limits(x))
        p_rot = sum(r.electrical_power(f[i], rho) for i, r in enumerate(self.vehicle.rotors))
        tip = max((r.tip_speed(rho, a) / a for r in self.vehicle.rotors), default=0.0) if a > 0 else 0.0
        R = quat_to_rot(x[6:10])
        on_ground = any((x[0:3] + R @ foot)[2] < 0.0 for foot in self.feet)
        return Outputs(rho, a, p_rot + self.vehicle.avionics_w + self.vehicle.heater_w, p_rot, tip, on_ground)
