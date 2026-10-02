"""
Error-state extended Kalman filter (ESKF) for navigation, after Sola (2017),
"Quaternion kinematics for the error-state Kalman filter", sec. 5-7.

Nominal state : p (world, m), v (world, m/s), q (body->world), b_a, b_g (IMU biases)
Error state   : dx = [dp, dv, dtheta, db_a, db_g]  (15), with q_true = q (x) Exp(dtheta)

Propagation runs on the IMU at the control rate; the altimeter, visual navigation
and barometer are fused as they arrive. Every update computes its normalised
innovation squared (NIS = y' S^-1 y), which is the residual the SADPF uses to
detect and isolate sensor faults. Measurements whose NIS exceeds a chi-square
gate are rejected so a faulty sensor cannot drag the estimate away before it is
isolated.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from .dynamics import cross3, euler_from_quat, quat_to_rot
from .sensors import Measurements, SensorConfig, vote

# 99.9 % chi-square quantiles for 1 and 3 degrees of freedom
CHI2_999 = {1: 10.83, 3: 16.27}


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _quat_exp(theta: np.ndarray) -> np.ndarray:
    a = np.linalg.norm(theta)
    if a < 1e-12:
        return np.array([1.0, *(0.5 * theta)])
    return np.array([np.cos(a / 2), *(np.sin(a / 2) * theta / a)])


@dataclass
class UpdateRecord:
    nis: float = 0.0
    dof: int = 1
    accepted: bool = True
    t: float = 0.0


class NavigationEKF:
    def __init__(self, gravity: float, x0: np.ndarray, sensors: SensorConfig, baro_sigma: float):
        self.g = np.array([0.0, 0.0, -gravity])
        self.cfg = sensors
        self.p = x0[0:3].copy()
        self.v = x0[3:6].copy()
        self.q = x0[6:10].copy()
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        c = sensors
        self.P = np.diag(np.r_[np.full(3, 0.05 ** 2), np.full(3, 0.02 ** 2), np.full(3, np.radians(1.0) ** 2),
                               np.full(3, (2 * c.accel_bias_sigma0) ** 2), np.full(3, (2 * c.gyro_bias_sigma0) ** 2)])
        self.baro_sigma = baro_sigma
        self.disabled: set[str] = set()  # sensors isolated by the SADPF
        self.last: Dict[str, UpdateRecord] = {}
        self.omega = np.zeros(3)  # bias-corrected body rate (for the controller)
        self.acc_body = np.zeros(3)  # bias-corrected specific force
        self._last_imu = (np.zeros(3), np.zeros(3))

    # ------------------------------------------------------------------ #
    def state_vector(self, x_true: np.ndarray) -> np.ndarray:
        """The true state with position, velocity, attitude and rates replaced by estimates."""
        xe = x_true.copy()
        xe[0:3], xe[3:6], xe[6:10], xe[10:13] = self.p, self.v, self.q, self.omega
        return xe

    def predict(self, accel: Optional[np.ndarray], gyro: Optional[np.ndarray], dt: float):
        if accel is None or gyro is None:  # every IMU unit out: hold the last sample
            accel, gyro = self._last_imu
        self._last_imu = (accel, gyro)
        c = self.cfg
        a_b = accel - self.ba
        w_b = gyro - self.bg
        self.omega, self.acc_body = w_b, a_b
        R = quat_to_rot(self.q)
        a_w = R @ a_b + self.g
        self.p = self.p + self.v * dt + 0.5 * a_w * dt * dt
        self.v = self.v + a_w * dt
        self.q = _quat_mul(self.q, _quat_exp(w_b * dt))
        self.q /= np.linalg.norm(self.q)

        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R @ _skew(a_b) * dt
        F[3:6, 9:12] = -R * dt
        F[6:9, 6:9] = np.eye(3) - _skew(w_b) * dt
        F[6:9, 12:15] = -np.eye(3) * dt
        Q = np.zeros((15, 15))
        Q[3:6, 3:6] = np.eye(3) * (c.accel_noise * dt) ** 2
        Q[6:9, 6:9] = np.eye(3) * (c.gyro_noise * dt) ** 2
        Q[9:12, 9:12] = np.eye(3) * c.accel_bias_rw ** 2 * dt
        Q[12:15, 12:15] = np.eye(3) * c.gyro_bias_rw ** 2 * dt
        self.P = F @ self.P @ F.T + Q

    # ------------------------------------------------------------------ #
    def _update(self, name: str, t: float, y: np.ndarray, H: np.ndarray, Rm: np.ndarray, gate: bool = True):
        S = H @ self.P @ H.T + Rm
        S_inv = np.linalg.inv(S)
        nis = float(y @ S_inv @ y)
        dof = len(y)
        accepted = (not gate) or nis <= CHI2_999[dof]
        self.last[name] = UpdateRecord(nis=nis, dof=dof, accepted=accepted, t=t)
        if not accepted:
            return
        K = self.P @ H.T @ S_inv
        dx = K @ y
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.q = _quat_mul(self.q, _quat_exp(dx[6:9]))
        self.q /= np.linalg.norm(self.q)
        self.ba += dx[9:12]
        self.bg += dx[12:15]

    def correct(self, m: Measurements):
        c = self.cfg
        R = quat_to_rot(self.q)
        if m.nav_pos is not None and "nav_pos" not in self.disabled:
            H = np.zeros((3, 15))
            H[:, 0:3] = np.eye(3)
            self._update("nav_pos", m.t, m.nav_pos - self.p, H, np.diag(np.square(c.nav_pos_noise)))
        if m.nav_yaw is not None and "nav_yaw" not in self.disabled:
            yaw = euler_from_quat(self.q)[2]
            y = np.array([(m.nav_yaw - yaw + np.pi) % (2 * np.pi) - np.pi])
            H = np.zeros((1, 15))
            H[0, 6:9] = R[2, :]  # small-tilt: yaw error = world-z component of the body rotation error
            self._update("nav_yaw", m.t, y, H, np.array([[np.radians(c.nav_yaw_noise_deg) ** 2]]))
        if m.altimeter is not None and "altimeter" not in self.disabled and R[2, 2] > 0.5:
            r33 = R[2, 2]
            pz = self.p[2]
            H = np.zeros((1, 15))
            H[0, 2] = 1.0 / r33
            H[0, 6] = -pz / r33 ** 2 * (-R[2, 1])
            H[0, 7] = -pz / r33 ** 2 * (R[2, 0])
            y = np.array([m.altimeter - pz / r33])
            self._update("altimeter", m.t, y, H, np.array([[c.altimeter_noise ** 2]]))
        if m.baro_alt is not None and "baro" not in self.disabled and np.isfinite(self.baro_sigma):
            H = np.zeros((1, 15))
            H[0, 2] = 1.0
            self._update("baro", m.t, np.array([m.baro_alt - self.p[2]]), H, np.array([[self.baro_sigma ** 2]]))

    def fused_imu(self, m: Measurements):
        acc_x = {int(s.split("#")[1]) for s in self.disabled if s.startswith("accel#")}
        gyr_x = {int(s.split("#")[1]) for s in self.disabled if s.startswith("gyro#")}
        return vote(m.accel_units, acc_x), vote(m.gyro_units, gyr_x)

    def step(self, m: Measurements, dt: float):
        self.predict(*self.fused_imu(m), dt)
        self.correct(m)

    def inflate(self, pos_sigma: float = 1.0, vel_sigma: float = 1.0):
        """Divergence recovery: re-open position/velocity uncertainty so the aiding sensors are accepted again."""
        self.P[0:3, :] = 0.0
        self.P[:, 0:3] = 0.0
        self.P[3:6, :] = 0.0
        self.P[:, 3:6] = 0.0
        self.P[0:3, 0:3] = np.eye(3) * pos_sigma ** 2
        self.P[3:6, 3:6] = np.eye(3) * vel_sigma ** 2

    @property
    def position_sigma(self) -> np.ndarray:
        return np.sqrt(np.diag(self.P)[0:3])
