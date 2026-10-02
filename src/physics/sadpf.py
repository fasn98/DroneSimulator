"""
SADPF - Sistema de Autodiagnostico e Prognostico de Falhas (thesis Ch. 5.4), the
part of it that can be implemented and verified in the Twin today:

  * model-based diagnosis (5.4.1): the measured motion is compared with what the
    on-board model predicts from the commands; the difference (the residual) is
    the fault signature;
  * criticality levels (5.4.2): Nivel 1 (aviso), Nivel 2 (alerta, autonomous
    corrective action), Nivel 3 (critico, drastic action);
  * autonomous actions: isolate a faulty sensor, reconfigure the control
    allocation around a degraded rotor (the "reconfigurar a matriz de controle" of
    the degraded flight modes), and land when the mission can no longer be flown.

Actuator faults (loss of effectiveness of one rotor) are detected from force and
moment residuals:
    r_F = m * a_z,meas - sum_i e_i f_i               (body z force)
    r_M = I * d(omega)/dt + omega x I omega - M(f)    (roll / pitch moments)
where f_i is the thrust predicted by a first-order model of each actuator driven
by the commands, a and omega come from the bias-corrected IMU (EKF), and both
sides pass through the same low-pass filter. A deficit D on rotor i produces
r = -D * [1, y_i, -x_i] (moments about the CG), so the rotor is isolated by the
column that best fits the residual, and its effectiveness is estimated from D.

Sensor faults are detected from the EKF innovations: repeated chi-square gate
rejections (NIS above the 99.9 % quantile) isolate the sensor, and a reading that
stops changing at all (zero variance over consecutive samples) is flagged as
stuck - real sensors always carry noise.

Honest scope: the detector uses the same physics as the simulated plant. The
Monte Carlo campaign therefore perturbs the plant (mass, inertia, per-rotor thrust
scale, actuator time constant) so the detector is evaluated with model mismatch.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from scipy.optimize import linprog

from .atmosphere import Body
from .vehicle import Vehicle

LEVEL_NAMES = {0: "Nominal", 1: "Nível 1 (aviso)", 2: "Nível 2 (alerta)", 3: "Nível 3 (crítico)"}


@dataclass
class RotorFault:
    """Loss of effectiveness of rotor `index`: from `t_start` it delivers `effectiveness` x command."""
    index: int
    effectiveness: float
    t_start: float


@dataclass
class SadpfEvent:
    t: float
    level: int
    code: str
    message: str
    data: Dict = field(default_factory=dict)


@dataclass
class SadpfConfig:
    residual_tau: float = 0.3  # s, low-pass on both sides of the residual
    detect_threshold: float = 0.035  # residual norm in fractions of weight (F) / weight x arm (M)
    detect_persistence: float = 0.3  # s above threshold before declaring a fault
    isolate_persistence: float = 0.5  # s with the same rotor best-fitting before isolating it
    min_height: float = 1.0  # m above the legs before actuator diagnosis is armed
    reconfigure: bool = True  # Nivel >= 2 rotor fault -> update the allocator
    land_on_critical: bool = True
    land_rate: float = 0.8  # m/s, emergency descent rate
    # Prognosis after a rotor fault: the thrust-to-weight still available with roll and pitch
    # balanced (yaw may be sacrificed). Below this margin the vehicle can hover but not
    # manoeuvre or reject gusts, so the fault is critical (Nivel 3) and the SADPF lands.
    min_post_fault_tw: float = 1.1
    nis_reject_count: Dict[str, int] = field(default_factory=lambda: {
        "nav_pos": 5, "nav_yaw": 5, "altimeter": 10, "baro": 10})
    stuck_samples: int = 6
    gyro_bias_spec: float = 0.02  # rad/s, Nivel 1 warning when the estimated bias exceeds it
    imu_vote_tau: float = 0.3  # s, low-pass on each unit's deviation from the median
    imu_vote_threshold: Dict[str, float] = field(default_factory=lambda: {"gyro": 0.015, "accel": 0.15})
    imu_vote_persistence: float = 0.3  # s


class ActuatorFDI:
    def __init__(self, vehicle: Vehicle, body: Body, cfg: SadpfConfig, dt: float):
        self.v, self.body, self.cfg, self.dt = vehicle, body, cfg, dt
        n = len(vehicle.rotors)
        self.n = n
        self.pos = np.array([r.position for r in vehicle.rotors]) if n else np.zeros((0, 3))
        self.tau_act = np.array([r.time_constant for r in vehicle.rotors])
        self.arm = max([np.linalg.norm(p[:2]) for p in self.pos] + [0.1])
        self.f_model = np.zeros(n)
        self.eff = np.ones(n)  # current belief (updated after isolation)
        self.alpha = 1.0 - np.exp(-dt / cfg.residual_tau)
        self.lp_force_meas = 0.0
        self.lp_force_exp = 0.0
        self.lp_omega = np.zeros(3)
        self.lp_m_exp = np.zeros(3)
        self.residual = np.zeros(3)  # normalised [F, Mx, My]
        self.armed = False
        self._above_since: Optional[float] = None
        self._cand: Optional[int] = None
        self._cand_since: Optional[float] = None
        self._estimates: List[float] = []
        self.candidate_loe = 0.0

    def _lp(self, old, new):
        return old + self.alpha * (new - old)

    def update(self, t: float, cmd: np.ndarray, acc_body: np.ndarray, omega: np.ndarray,
               mass: float, height: float, gear_height: float) -> Optional[Dict]:
        """Advance the actuator model with the command held over the last cycle and test the residual.

        Returns a dict {rotor, loe} when a rotor fault is isolated this cycle.
        """
        if self.n == 0:
            return None
        rho, a = self.body.density(height), self.body.speed_of_sound(height)
        lim = np.array([r.max_thrust(rho, a) for r in self.v.rotors])
        u = np.clip(cmd[:self.n], 0.0, lim)
        self.f_model += (u - self.f_model) * (1.0 - np.exp(-self.dt / self.tau_act))
        f = self.f_model * self.eff
        I = self.v.inertia
        m_exp = np.array([np.sum(self.pos[:, 1] * f), -np.sum(self.pos[:, 0] * f), 0.0])
        m_exp = m_exp - np.cross(omega, I @ omega)  # move the gyroscopic term to the model side
        omega_prev = self.lp_omega.copy()
        self.lp_omega = self._lp(self.lp_omega, omega)
        self.lp_m_exp = self._lp(self.lp_m_exp, m_exp)
        self.lp_force_meas = self._lp(self.lp_force_meas, mass * acc_body[2])
        self.lp_force_exp = self._lp(self.lp_force_exp, np.sum(f))
        w = mass * self.body.gravity
        m_meas = I @ ((self.lp_omega - omega_prev) / self.dt)
        r_m = (m_meas - self.lp_m_exp) / (w * self.arm)
        self.residual = np.array([(self.lp_force_meas - self.lp_force_exp) / w, r_m[0], r_m[1]])

        was_armed = self.armed
        self.armed = height > gear_height + self.cfg.min_height
        if not self.armed or not was_armed:
            self._above_since = self._cand = self._cand_since = None
            self._estimates = []
            return None

        if np.linalg.norm(self.residual) < self.cfg.detect_threshold:
            self._above_since = self._cand = self._cand_since = None
            self._estimates = []
            return None
        if self._above_since is None:
            self._above_since = t
        if t - self._above_since < self.cfg.detect_persistence:
            return None
        # isolation: deficit D_i (fraction of weight) that best explains the residual
        best, best_err, best_d = None, np.inf, 0.0
        for i in range(self.n):
            b = np.array([1.0, self.pos[i, 1] / self.arm, -self.pos[i, 0] / self.arm])
            d = -float(b @ self.residual) / float(b @ b)
            if d <= 0:
                continue
            err = np.linalg.norm(self.residual + d * b)
            if err < best_err:
                best, best_err, best_d = i, err, d
        if best is None:
            return None
        if best != self._cand:
            self._cand, self._cand_since, self._estimates = best, t, []
        thrust_i = max(self.f_model[best], 1e-6)
        # D * weight is the missing force; relative to the current command it is an effectiveness drop
        self._estimates.append(best_d * w / thrust_i)
        self.candidate_loe = float(np.median(self._estimates))
        if t - self._cand_since < self.cfg.isolate_persistence:
            return None
        new_eff = float(np.clip(self.eff[best] - self.candidate_loe, 0.0, 1.0))
        self.eff[best] = new_eff
        self._above_since = self._cand = self._cand_since = None
        self._estimates = []
        return {"rotor": best, "loe": 1.0 - new_eff, "effectiveness": new_eff}


class SensorMonitor:
    def __init__(self, cfg: SadpfConfig, dt: float):
        self.cfg = cfg
        self.dt = dt
        self.rejections: Dict[str, int] = {}
        self.history: Dict[str, List[np.ndarray]] = {}
        self.isolated: Dict[str, str] = {}  # sensor -> reason
        self.dev: Dict[str, np.ndarray] = {}
        self.above_since: Dict[str, float] = {}

    def _vote_check(self, t: float, kind: str, units: list, new: list):
        """Compare each healthy IMU unit with the median of the healthy units."""
        healthy = [k for k, u in enumerate(units) if u is not None and f"{kind}#{k}" not in self.isolated]
        if len(healthy) < 3:  # with two units a disagreement cannot say which one is wrong
            return
        med = np.median(np.array([units[k] for k in healthy]), axis=0)
        alpha = 1.0 - np.exp(-self.dt / self.cfg.imu_vote_tau)
        for k in healthy:
            key = f"{kind}#{k}"
            d = self.dev.get(key, np.zeros(3))
            d = d + alpha * ((units[k] - med) - d)
            self.dev[key] = d
            if np.linalg.norm(d) > self.cfg.imu_vote_threshold[kind]:
                start = self.above_since.setdefault(key, t)
                if t - start >= self.cfg.imu_vote_persistence:
                    self.isolated[key] = "vote"
                    new.append((key, "vote"))
            else:
                self.above_since.pop(key, None)

    def divergence_check(self, ekf, meas) -> bool:
        """Two independent aiding sensors rejected at the same time point to the filter, not to the
        sensors (two simultaneous sensor faults are far less likely): re-open the covariance."""
        for name, rec in ekf.last.items():
            if name in self.isolated or rec.t != meas.t:
                continue
            self.rejections[name] = 0 if rec.accepted else self.rejections.get(name, 0) + 1
        streaking = [n for n in ("nav_pos", "altimeter") if self.rejections.get(n, 0) >= 2 and n not in self.isolated]
        if len(streaking) >= 2:
            ekf.inflate()
            for n in streaking:
                self.rejections[n] = 0
            return True
        return False

    def update(self, ekf, meas, touchdown: bool = False) -> List[tuple]:
        """Returns newly isolated sensors as (sensor, reason). Call divergence_check first.

        During a touchdown the impact (which a 50 Hz IMU aliases) is what makes innovations jump,
        not the sensors, so rejections then re-open the covariance instead of isolating sensors."""
        new = []
        if touchdown:
            if any(self.rejections.get(n, 0) >= 2 for n in ("nav_pos", "altimeter")):
                ekf.inflate(0.3, 0.5)
                self.rejections = {}
        for name, rec in ekf.last.items():
            if touchdown or name in self.isolated or rec.t != meas.t:
                continue
            if self.rejections.get(name, 0) >= self.cfg.nis_reject_count.get(name, 10):
                self.isolated[name] = "innovation"
                new.append((name, "innovation"))
        self._vote_check(meas.t, "gyro", meas.gyro_units, new)
        self._vote_check(meas.t, "accel", meas.accel_units, new)
        readings = {"altimeter": meas.altimeter, "nav_pos": meas.nav_pos, "nav_yaw": meas.nav_yaw,
                    "baro": meas.baro_alt}
        for k, u in enumerate(meas.accel_units):
            readings[f"accel#{k}"] = u
        for k, u in enumerate(meas.gyro_units):
            readings[f"gyro#{k}"] = u
        for name, val in readings.items():
            if val is None or name in self.isolated:
                continue
            h = self.history.setdefault(name, [])
            h.append(np.atleast_1d(np.array(val, dtype=float)))
            if len(h) > self.cfg.stuck_samples:
                h.pop(0)
            if len(h) == self.cfg.stuck_samples and all(np.array_equal(h[0], x) for x in h[1:]):
                self.isolated[name] = "stuck"
                new.append((name, "stuck"))
        return new


SENSOR_LABEL = {"altimeter": "altímetro laser", "nav_pos": "navegação visual (posição)",
                "nav_yaw": "navegação visual (rumo)", "baro": "barômetro"}


def sensor_label(name: str) -> str:
    if "#" in name:
        kind, k = name.split("#")
        return f"{'giroscópio' if kind == 'gyro' else 'acelerômetro'} da IMU {int(k) + 1}"
    return SENSOR_LABEL.get(name, name)


class Sadpf:
    """Supervisor: runs the diagnosers, grades criticality and takes the autonomous actions."""

    def __init__(self, vehicle: Vehicle, body: Body, dt: float, cfg: SadpfConfig | None = None):
        self.cfg = cfg or SadpfConfig()
        self.vehicle = vehicle
        self.body = body
        self.fdi = ActuatorFDI(vehicle, body, self.cfg, dt)
        self.sensors = SensorMonitor(self.cfg, dt)
        self.landed = False
        self.divergence_resets = 0
        self._still_since: Optional[float] = None
        self.events: List[SadpfEvent] = []
        self.level = 0
        self.land_requested: Optional[float] = None
        self._gyro_warned = False

    def post_fault_thrust_to_weight(self, effectiveness: np.ndarray, height: float, mass: float) -> float:
        """Max vertical force with zero roll and pitch moment, divided by weight (linear program)."""
        rotors = self.vehicle.rotors
        if not rotors:
            return 0.0
        rho, a = self.body.density(height), self.body.speed_of_sound(height)
        umax = np.array([r.max_thrust(rho, a) for r in rotors])
        e = np.asarray(effectiveness, dtype=float)
        pos = np.array([r.position for r in rotors])
        res = linprog(-e, A_eq=np.vstack([e * pos[:, 1], e * pos[:, 0]]), b_eq=np.zeros(2),
                      bounds=list(zip(np.zeros(len(rotors)), umax)), method="highs")
        if not res.success:
            return 0.0
        return float(-res.fun / (mass * self.body.gravity))

    def _emit(self, t, level, code, message, **data):
        self.events.append(SadpfEvent(t, level, code, message, data))
        self.level = max(self.level, level)

    def update(self, t: float, cmd: np.ndarray, ekf, meas, mass: float, allocator=None):
        height = float(ekf.p[2])
        if self.sensors.divergence_check(ekf, meas):
            self.divergence_resets += 1
            if self.divergence_resets == 1 or self.divergence_resets % 10 == 0:
                self._emit(t, max(self.level, 1), "ekf_reset", "Resíduos simultâneos em sensores independentes: "
                           "incerteza do EKF reaberta (recuperação de divergência)", count=self.divergence_resets)
        # Land detector (as autopilots do): on the legs AND no vertical motion for a while, so the
        # attitude loop keeps the vehicle level through the touchdown bounce before motors are cut.
        if self.land_requested is not None and not self.landed:
            still = height < self.vehicle.gear_height + 0.15 and abs(float(ekf.v[2])) < 0.2
            if not still:
                self._still_since = None
            elif self._still_since is None:
                self._still_since = t
        if self._still_since is not None and not self.landed and t - self._still_since >= 1.0:
            self.landed = True
            ekf.inflate(0.5, 0.5)  # touchdown impacts alias in a 50 Hz IMU; let the aiding sensors re-anchor
            self._emit(t, self.level, "landed", "Pouso de emergência concluído; motores desligados")
        if self.landed:
            return
        iso = self.fdi.update(t, cmd, ekf.acc_body, ekf.omega, mass, height, self.vehicle.gear_height)
        if iso is not None:
            loe = iso["loe"]
            eff = self.fdi.eff.copy()
            tw_post = self.post_fault_thrust_to_weight(eff, max(height, 0.0), mass)
            level = 1 if loe < 0.25 else 2
            if tw_post < self.cfg.min_post_fault_tw:
                level = 3
            action = "monitorar tendência"
            if level >= 2 and self.cfg.reconfigure and allocator is not None:
                allocator.effectiveness[iso["rotor"]] = iso["effectiveness"]
                action = "alocação de controle reconfigurada"
            if level == 3 and self.cfg.land_on_critical and self.land_requested is None:
                self.land_requested = t
                action += f"; T/W restante {tw_post:.2f} < {self.cfg.min_post_fault_tw:.2f} — pouso de emergência"
            elif level == 2:
                action += f"; T/W restante {tw_post:.2f} — missão continua"
            self._emit(t, level, "rotor_loe",
                       f"Rotor {iso['rotor'] + 1}: perda de eficácia estimada {loe * 100:.0f}% — {action}",
                       rotor=iso["rotor"], loe=loe, tw_post=tw_post)
        touchdown = self.land_requested is not None and height < self.vehicle.gear_height + 1.5
        for name, reason in self.sensors.update(ekf, meas, touchdown):
            ekf.disabled.add(name)
            level = 3 if name == "nav_pos" else 2
            action = "sensor isolado; navegação continua com os demais"
            if level == 3 and self.cfg.land_on_critical and self.land_requested is None:
                self.land_requested = t
                action = "sensor isolado; sem referência de posição — pouso de emergência no local"
            if "#" in name:
                action = "unidade isolada; IMU segue com as redundantes"
            why = {"stuck": "leitura congelada", "vote": "diverge da votação das 3 IMUs"}.get(
                reason, "resíduo do EKF fora do limite χ²")
            self._emit(t, level, f"sensor_{name}", f"Falha: {sensor_label(name)} ({why}) — {action}",
                       sensor=name, reason=reason)
        if not self._gyro_warned and np.linalg.norm(ekf.bg) > self.cfg.gyro_bias_spec:
            self._gyro_warned = True
            self._emit(t, 1, "gyro_bias", f"Viés do giroscópio estimado em {np.degrees(np.linalg.norm(ekf.bg)):.2f} °/s "
                       "(acima da especificação; compensado pelo EKF)", bias=float(np.linalg.norm(ekf.bg)))

    @property
    def level_name(self) -> str:
        return LEVEL_NAMES[self.level]
