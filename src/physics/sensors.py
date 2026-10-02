"""
Sensor models for the Twin (phase 3).

The suite follows what a Mars rotorcraft can actually use (Ingenuity flew with an
IMU, a laser rangefinder, an inclinometer and a downward navigation camera):

    IMU (x3)         accelerometer (specific force) + gyro, white noise + bias random walk;
                     three independent units, as the thesis specifies ("tripla redundancia
                     para IMUs"), fused by median voting so one faulty unit is outvoted
    laser altimeter  slant range to flat ground along the body -z axis, valid 0.3..50 m
    visual nav       position and heading from terrain-relative navigation (camera)
    barometer        altitude from static pressure; noise is set in PASCAL, so the
                     altitude noise follows physics: sigma_h = sigma_p / (rho * g).
                     On Mars (rho*g ~ 0.057 Pa/m) 0.2 Pa of noise is ~3.5 m of altitude,
                     which is why Mars vehicles do not rely on barometric altimetry.

There is no magnetometer: Mars has no global magnetic field, so heading comes from
visual navigation (and, on the real vehicle, a sun sensor before take-off).

Sensor faults are injected per sensor with `SensorFault` (bias, stuck, dropout,
noise increase); the fault model is applied on top of the healthy reading.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .atmosphere import Body
from .dynamics import euler_from_quat, quat_to_rot

SENSORS = ("accel", "gyro", "altimeter", "nav_pos", "nav_yaw", "baro")


@dataclass
class SensorConfig:
    # IMU: per-sample standard deviations at the control rate, plus bias random walk densities.
    accel_noise: float = 0.05  # m/s^2 (vibration-dominated, not the datasheet floor)
    gyro_noise: float = 0.004  # rad/s
    accel_bias_sigma0: float = 0.02  # m/s^2, turn-on bias spread
    gyro_bias_sigma0: float = 0.002  # rad/s
    accel_bias_rw: float = 2e-4  # m/s^2/sqrt(s)
    gyro_bias_rw: float = 2e-5  # rad/s/sqrt(s)
    altimeter_noise: float = 0.03  # m
    altimeter_range: tuple = (0.3, 50.0)  # m, valid slant range
    altimeter_rate_hz: float = 25.0
    nav_pos_noise: tuple = (0.25, 0.25, 0.5)  # m (x, y, z)
    nav_yaw_noise_deg: float = 1.0
    nav_rate_hz: float = 5.0
    baro_pressure_noise_pa: float = 0.2
    baro_rate_hz: float = 10.0
    imu_count: int = 3


@dataclass
class SensorFault:
    """A fault on one sensor channel, active from `t_start` (s).

    kind:
        "bias"     add `value` (scalar or vector, sensor units) to the reading
        "stuck"    freeze the reading at its value when the fault starts
        "dropout"  no reading at all
        "noise"    multiply the noise by `value`
    """
    sensor: str
    kind: str
    t_start: float
    value: float | np.ndarray = 0.0
    unit: int = 0  # which IMU (accel / gyro faults only)
    _frozen: Optional[np.ndarray] = field(default=None, repr=False)

    def __post_init__(self):
        if self.sensor not in SENSORS:
            raise ValueError(f"unknown sensor {self.sensor!r}; expected one of {SENSORS}")
        if self.kind not in ("bias", "stuck", "dropout", "noise"):
            raise ValueError(f"unknown sensor fault kind {self.kind!r}")


def vote(units: np.ndarray, excluded: set) -> Optional[np.ndarray]:
    """Redundancy management: median of the healthy units (mean of two, or the last one)."""
    rows = [u for k, u in enumerate(units) if k not in excluded and u is not None]
    if not rows:
        return None
    rows = np.array(rows)
    return np.median(rows, axis=0) if len(rows) >= 3 else rows.mean(axis=0)


@dataclass
class Measurements:
    """One control-cycle bundle. A channel is None when it produced no sample this cycle.

    accel_units / gyro_units hold one row per IMU (None for a unit in dropout).
    """
    t: float
    accel_units: list
    gyro_units: list
    altimeter: Optional[float] = None
    nav_pos: Optional[np.ndarray] = None
    nav_yaw: Optional[float] = None
    baro_alt: Optional[float] = None


class SensorSuite:
    def __init__(self, body: Body, rng: np.random.Generator, config: SensorConfig | None = None):
        self.cfg = config or SensorConfig()
        self.body = body
        self.rng = rng
        c = self.cfg
        self.accel_bias = rng.normal(0.0, c.accel_bias_sigma0, (c.imu_count, 3))
        self.gyro_bias = rng.normal(0.0, c.gyro_bias_sigma0, (c.imu_count, 3))
        self.faults: List[SensorFault] = []
        self._next = {"altimeter": 0.0, "nav": 0.0, "baro": 0.0}
        self._p0 = body.pressure(0.0)

    # ------------------------------------------------------------------ #
    def baro_altitude_noise(self, altitude: float = 0.0) -> float:
        """1-sigma barometric altitude noise (m): sigma_p / (rho g). Infinite in vacuum."""
        rho = self.body.density(altitude)
        return self.cfg.baro_pressure_noise_pa / (rho * self.body.gravity) if rho > 0 else float("inf")

    def _apply(self, name: str, t: float, value, unit: int = 0):
        """Return (value, noise multiplier) after faults, or (None, _) on dropout."""
        mult = 1.0
        for f in self.faults:
            if f.sensor != name or t < f.t_start or (name in ("accel", "gyro") and f.unit != unit):
                continue
            if f.kind == "dropout":
                return None, mult
            if f.kind == "stuck":
                if f._frozen is None:
                    f._frozen = np.array(value, dtype=float, copy=True)
                return (f._frozen.copy() if np.ndim(f._frozen) else float(f._frozen)), 0.0
            if f.kind == "bias":
                value = value + f.value
            if f.kind == "noise":
                mult *= float(f.value)
        return value, mult

    def sample(self, t: float, dt: float, x: np.ndarray, specific_force_body: np.ndarray) -> Measurements:
        c, rng = self.cfg, self.rng
        n = c.imu_count
        self.accel_bias += rng.normal(0.0, c.accel_bias_rw * np.sqrt(dt), (n, 3))
        self.gyro_bias += rng.normal(0.0, c.gyro_bias_rw * np.sqrt(dt), (n, 3))
        accs, gyrs = [], []
        for k in range(n):
            acc, m = self._apply("accel", t, specific_force_body + self.accel_bias[k], k)
            accs.append(None if acc is None else acc + m * rng.normal(0.0, c.accel_noise, 3))
            gyr, m = self._apply("gyro", t, x[10:13] + self.gyro_bias[k], k)
            gyrs.append(None if gyr is None else gyr + m * rng.normal(0.0, c.gyro_noise, 3))
        meas = Measurements(t=t, accel_units=accs, gyro_units=gyrs)

        R = quat_to_rot(x[6:10])
        if t >= self._next["altimeter"]:
            self._next["altimeter"] = t + 1.0 / c.altimeter_rate_hz
            cos_tilt = R[2, 2]
            if cos_tilt > 0.5:
                rng_true = x[2] / cos_tilt
                if c.altimeter_range[0] <= rng_true <= c.altimeter_range[1]:
                    val, m = self._apply("altimeter", t, rng_true)
                    if val is not None:
                        meas.altimeter = float(val + m * rng.normal(0.0, c.altimeter_noise))
        if t >= self._next["nav"]:
            self._next["nav"] = t + 1.0 / c.nav_rate_hz
            val, m = self._apply("nav_pos", t, x[0:3].copy())
            if val is not None:
                meas.nav_pos = val + m * rng.normal(0.0, 1.0, 3) * np.asarray(c.nav_pos_noise)
            yaw = euler_from_quat(x[6:10])[2]
            val, m = self._apply("nav_yaw", t, yaw)
            if val is not None:
                meas.nav_yaw = float(val + m * rng.normal(0.0, np.radians(c.nav_yaw_noise_deg)))
        if t >= self._next["baro"] and self.body.density(0.0) > 0:
            self._next["baro"] = t + 1.0 / c.baro_rate_hz
            val, m = self._apply("baro", t, x[2])
            if val is not None:
                meas.baro_alt = float(val + m * rng.normal(0.0, self.baro_altitude_noise(x[2])))
        return meas
