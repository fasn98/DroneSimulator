"""
Flight procedures (the "pilot") for the helicopter scenarios.

Each procedure is a guidance function  fn(t, x) -> (position_ref, velocity_ref, yaw_ref)  for the SAS,
plus mode changes on the simulator (autorotation, cushion, collective down after touchdown). After an
engine failure the procedure only reacts to the SADPF detection event, after a pilot reaction time,
and follows the action the SADPF recommended: nothing is scheduled on the known failure time.

Model criteria and constants are labelled; regulatory references are in docs/helicoptero-uti.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .isa import G0
from .rotor import best_speeds, level_flight
from .model import engine_limit_w

KT = 0.514444
FT = 0.3048
PILOT_DELAY = 1.0  # s, recognition + reaction after the SADPF alert (ESTIMADO, order used in Cat A analyses)
ROC_CAT_A = 100 * FT / 60  # 14 CFR 29.67(a)(1): >= 100 ft/min at VTOSS, OEI 2-min power, OGE


def _sadpf_action(sim, t: float) -> Optional[str]:
    """Recommended action once the pilot has had PILOT_DELAY to react, else None."""
    s = sim.sadpf
    if s is None or s.detect_t is None or t < s.detect_t + PILOT_DELAY:
        return None
    return s.recommendation_code


def _kv(sim) -> float:
    """Velocity-loop gain of the SAS (acceleration per m/s of velocity error)."""
    return 2.0 * sim.g.pos_zeta * sim.g.pos_wn


def oei_climb_rate(p, atm, z: float, mass: float, v: float) -> float:
    """Steady OEI rate of climb (m/s) at true airspeed v with OEI 2-min power, out of ground effect.

    Energy method (DERIVADO): ROC = eta (P_OEI2 - P_required) / W.
    """
    rho = atm.density(z)
    p_av = engine_limit_w(p, atm, z, "OEI2", 1)
    p_req = level_flight(p, v, rho, mass).p_engines
    return p.eta_tr * (p_av - p_req) / (mass * G0)


def vtoss(p, atm, z: float, mass: float, v_min_kt: float = 25.0, v_max_kt: float = 80.0) -> Optional[float]:
    """Lowest speed (kt, >= v_min_kt) at which the OEI 2-min climb meets 100 ft/min; None if no speed does."""
    for v_kt in np.arange(v_min_kt, v_max_kt + 0.1, 1.0):
        if oei_climb_rate(p, atm, z, mass, v_kt * KT) >= ROC_CAT_A:
            return float(v_kt)
    return None


def min_descent_speed(p, atm, z: float, mass: float) -> Dict[str, float]:
    """Autorotation: speed of minimum rate of descent and the steady descent rate there (energy method,
    DERIVADO): the rotor must supply main + tail + accessory power from the descent, W Vz = P(V)."""
    rho = atm.density(z)

    def rod(v):
        tp = level_flight(p, v, rho, mass)
        return (tp.p_main + tp.p_tail + tp.p_acc) / (mass * G0)
    vs = np.arange(10.0, 140.0, 1.0) * KT
    r = [rod(v) for v in vs]
    i = int(np.argmin(r))
    return {"v_md_kt": float(vs[i] / KT), "rod_ms": float(r[i]), "rod_hover_ms_momentum": float(rod(0.0))}


# ------------------------------------------------------------------------------------------------
@dataclass
class CatATakeoff:
    """Category A vertical take-off from an elevated heliport (model of the procedure, not an RFM procedure).

    Heights are of the skids above the deck (CG height minus the CG-to-skid distance).
    AEO: vertical climb to the TDP above the deck, then accelerate forward climbing.
    Engine failure (SADPF) before the TDP -> reject: back onto the deck with OEI power.
    After the TDP -> continue: nose down, accelerate to VTOSS (height loss allowed below the deck once clear
    of it), then climb at VTOSS. Speeds are airspeeds; the ground-speed references account for the wind.
    """
    sim: object
    tdp_height: float = 12.0  # m of skid height above the deck (ESTIMADO, configurable)
    vtoss_kt: float = 40.0  # set by the caller from vtoss()
    climb_rate: float = 1.5
    accel: float = 1.5  # m/s^2 AEO acceleration after the TDP
    force_action: Optional[str] = None  # "abortar"/"prosseguir": fly this branch whatever the SADPF recommends
    advisor: Optional[object] = None  # advisory.BranchPredictor, called once at the SADPF detection
    follow_advisory: bool = False  # only for studies: fly the branch the advisory recommends (default: display)
    phase: str = "vertical"
    tdp_t: Optional[float] = None
    _v: float = 0.0
    _last_t: float = 0.0
    log: List[dict] = field(default_factory=list)

    def skid_height(self, x) -> float:
        return float(x[2]) - self.sim.p.cg_h

    def __call__(self, t: float, x: np.ndarray):
        sim = self.sim
        dt = max(t - self._last_t, 0.0)
        self._last_t = t
        if self.advisor is not None and sim.sadpf is not None and sim.sadpf.detect_t is not None \
                and getattr(self, "_advised", None) is None:
            rec = sim.sadpf.recommendation_code
            proc_action = self.force_action or (rec if rec in ("abortar", "prosseguir") else None)
            self._advised = self.advisor(sim, self, proc_action) if proc_action else {}
            if self.follow_advisory and self._advised and self._advised.get("advise"):
                self.force_action = self._advised["advise"]
        action = _sadpf_action(sim, t)
        if action in ("abortar", "prosseguir") and self.force_action:
            action = self.force_action
        pad = sim.dyn.terrain
        h = self.skid_height(x)
        wind_x = float(sim.dyn.wind[0])
        if self.phase in ("vertical", "accelerate") and action in ("abortar", "prosseguir"):
            self.phase = "reject" if action == "abortar" else "continue"
            self.log.append({"t": t, "phase": self.phase, "skid_h": h})
        if self.phase == "vertical":
            sim.flight_phase = "catA_pre_tdp"
            if h >= self.tdp_height:
                self.phase, self.tdp_t = "accelerate", t
                sim.flight_phase = "catA_post_tdp"
                self.log.append({"t": t, "phase": "tdp", "skid_h": h})
            zr = sim.p.cg_h + min(self.tdp_height + 0.5, max(h, 0.0) + self.climb_rate * 1.0)
            return np.array([pad.pad_x, pad.pad_y, zr]), np.array([0.0, 0.0, self.climb_rate]), 0.0
        v_air = float(x[3]) - wind_x
        if self.phase == "accelerate":
            self._v = min(max(self._v, v_air) + self.accel * dt, 65 * KT)
            return np.array([x[0], pad.pad_y, x[2] + 1.0]), np.array([self._v + wind_x, 0.0, 1.5]), 0.0
        if self.phase == "reject":
            vz = -1.0 if h > 3.0 else -0.3
            if sim.dyn.aero(x, sim.u)[3].on_ground:
                sim.collective_hold = sim.p.theta_min
                if self.log[-1]["phase"] != "landed":
                    self.log.append({"t": t, "phase": "landed"})
            return np.array([pad.pad_x, pad.pad_y, x[2] + vz]), np.array([0.0, 0.0, vz]), 0.0
        # continue: accelerate to VTOSS holding height if the power allows, then climb at VTOSS
        v_t = self.vtoss_kt * KT
        self._v = min(max(self._v, v_air) + 2.5 * dt, v_t)
        vz = 0.0 if v_air < 0.95 * v_t else 1.0
        sim.g.max_tilt_deg = 15.0
        return np.array([x[0], pad.pad_y, x[2] + vz]), np.array([self._v + wind_x, 0.0, vz]), 0.0


CRITERIA = ("elevado_29_60", "literal_29_59c")
CLEAR_EDGE = 15 * FT  # 14 CFR 29.60(a)(2): every part clears all obstacles by >= 15 ft when clearing the edge
CLEAR_OBST = 35 * FT  # EASA CAT.POL.H.205(b)(4): obstacles cleared by >= 10,7 m (35 ft) in the continued take-off
CLEAR_LITERAL = 15 * FT  # 14 CFR 29.59(c): not below 15 ft above the take-off surface (TDP above 15 ft)


def evaluate_cat_a(tel, sim, vtoss_kt: Optional[float], branch: str,
                   criterion: str = "elevado_29_60") -> Dict[str, object]:
    """Model safety criteria of each branch (see docs/helicoptero-uti.md, Categoria A).

    reject:   back on the deck (1 m inside the edge), touchdown sink <= 1.5 m/s, no roll-over (ESTIMADO limits)
    continue: VTOSS reached, no contact, steady OEI 2-min climb at VTOSS >= 100 ft/min (29.67(a)(1)), and
      criterion "literal_29_59c":  skids never below 15 ft above the deck level after the failure (29.59(c));
      criterion "elevado_29_60":   skids >= 15 ft above the deck while over it and when crossing its edge
                                   (29.60(a)(2), descent below the deck allowed after that), then >= 35 ft
                                   above the ground below, treated as the obstacle (CAT.POL.H.205(b)(4)).
    """
    assert criterion in CRITERIA, criterion
    pad = sim.dyn.terrain
    cg_h = sim.p.cg_h
    t, x, y, z, vz = (tel.column(k) for k in ("t", "x", "y", "z", "vz"))
    h_skid = z - cg_h  # skid height relative to the deck
    agl_skid = tel.column("agl") - cg_h
    ground = tel.column("on_ground") > 0
    idx = np.arange(len(t))
    res: Dict[str, object] = {"branch": branch}
    airborne = np.where(~ground)[0]
    if len(airborne) == 0:
        return {**res, "safe": False, "reason": "não decolou"}
    if branch == "reject":
        # the touchdown back on the deck: the landing that ends the flight, after the engine failure and after the
        # aircraft has been airborne for 1 s (a skid bounce at lift-off is never the touchdown); see find_touchdown
        td = find_touchdown(t, ground, vz, agl_skid, t_from=sim.fail_time, min_air_s=1.0)
        if td is None:
            return {**res, "safe": False, "reason": "não pousou"}
        k = td["k"]
        sink = td["sink_ms"]
        res.update(touchdown_t=td["t"], n_bounces=td["n_bounces"])
        on_pad = pad.on_pad(x[k], y[k], margin=1.0) and abs(h_skid[k]) < 1.0
        tilt = float(np.max(np.hypot(tel.column("roll_deg"), tel.column("pitch_deg"))[k:]))
        res.update(touchdown_sink_ms=sink, on_pad=bool(on_pad), max_tilt_after_deg=tilt, outcome=classify_touchdown(sink),
                   min_nr_pct=float(tel.column("nr_pct")[airborne[0]:].min()))
        res["safe"] = bool(on_pad and sink <= 1.5 and tilt < 15.0)
        res["reason"] = "ok" if res["safe"] else ("fora do heliponto" if not on_pad else
                                                   f"toque a {sink:.1f} m/s (> 1,5 m/s)" if sink > 1.5 else "tombamento")
        return res
    edge = pad.pad_x + pad.pad_half_size
    t0 = sim.fail_time if sim.fail_time is not None else t[airborne[0]]
    after = (idx > airborne[0]) & (t >= t0) & ~ground  # judged from the engine failure on
    over = after & (x <= edge)  # over the deck, up to the edge crossing
    beyond = after & (x > edge)
    min_over_deck = float(np.min(h_skid[over])) if np.any(over) else float("nan")
    min_agl_beyond = float(np.min(agl_skid[beyond])) if np.any(beyond) else float("nan")
    min_rel_deck = float(np.min(h_skid[after])) if np.any(after) else float("nan")
    tas = tel.column("tas_ms")
    reached = bool(vtoss_kt is not None and np.any(tas[after] >= 0.95 * vtoss_kt * KT))
    touched = bool(np.any(ground & (idx > airborne[0]) & (t >= t0)))
    roc = oei_climb_rate(sim.p, sim.atm, 0.0, sim.dyn.mass(sim.x), vtoss_kt * KT) if vtoss_kt else -1.0
    res.update(criterion=criterion, min_height_over_deck_m=min_over_deck, min_agl_beyond_m=min_agl_beyond,
               min_height_rel_deck_m=min_rel_deck, reached_vtoss=reached, touched_down=touched,
               oei_roc_at_vtoss_fpm=roc / FT * 60, vtoss_kt=vtoss_kt,
               max_drop_below_deck_m=float(max(0.0, -min_rel_deck)) if np.any(after) else 0.0)  # 29.60(a)(3)
    if vtoss_kt is None:
        res.update(safe=False, reason="nenhuma velocidade atinge 100 ft/min OEI")
    elif touched:
        res.update(safe=False, reason="tocou o heliponto ou o solo")
    elif criterion == "literal_29_59c" and min_rel_deck < CLEAR_LITERAL:
        res.update(safe=False, reason=f"desceu a {min_rel_deck:.1f} m do nível do deck (< 15 ft, 29.59(c))")
    elif criterion == "elevado_29_60" and not math.isnan(min_over_deck) and min_over_deck < CLEAR_EDGE:
        res.update(safe=False, reason=f"cruzou a borda a {min_over_deck:.1f} m acima do deck (< 15 ft, 29.60)")
    elif criterion == "elevado_29_60" and not math.isnan(min_agl_beyond) and min_agl_beyond < CLEAR_OBST:
        res.update(safe=False, reason=f"passou a {min_agl_beyond:.1f} m do solo (< 35 ft)")
    elif not reached:
        res.update(safe=False, reason="não atingiu VTOSS")
    elif roc < ROC_CAT_A:
        res.update(safe=False, reason="subida OEI na VTOSS < 100 ft/min")
    else:
        res.update(safe=True, reason="ok")
    return res


# ------------------------------------------------------------------------------------------------
@dataclass
class OeiLanding:
    """One engine out, land straight ahead (the manoeuvre that defines the height-velocity envelope, 29.87(a)).

    After the SADPF alert and the pilot delay (technique of the model, constants ESTIMADO):
      approach: fly at v_app = v_oei + margin (v_oei = lowest speed at which OEI 30 s power holds level flight;
                faster: decelerate level first; slower and high enough: trade height for speed), descending on a
                height-scheduled sink rate down to flare_h;
      flare:    decelerate at flare_decel holding a slow sink into ground effect;
      cushion:  below cushion_h the collective controls the sink rate and may spend rotor energy (no NR droop
                protection), until touchdown.
    """
    sim: object
    v_oei: float = 9.0  # m/s, set by the caller
    v_margin: float = 3.0  # m/s above v_oei for the approach (ESTIMADO)
    sink_k: float = 0.15  # 1/s, sink-rate schedule (ESTIMADO)
    sink_max: float = 3.0  # m/s (ESTIMADO)
    flare_h: float = 5.0  # m skid height where the final deceleration starts (ESTIMADO)
    flare_decel: float = 1.5  # m/s^2 (ESTIMADO)
    flare_slow: float = 5.0  # m/s: above it the flare holds flare_h instead of descending (ESTIMADO)
    cushion_h: float = 2.0  # m skid height (ESTIMADO)
    phase: str = "cruise"
    hold: Optional[np.ndarray] = None
    log: List[dict] = field(default_factory=list)

    def __call__(self, t: float, x: np.ndarray):
        sim = self.sim
        h = float(x[2]) - sim.dyn.terrain.height(x[0], x[1]) - sim.p.cg_h
        v = float(x[3])
        if self.hold is None:
            self.hold = x[0:3].copy()
            self._v0 = v
        if self.phase == "cruise":
            if _sadpf_action(sim, t) is not None:
                self.phase = "approach" if h > self.flare_h else "flare"
                self.log.append({"t": t, "phase": self.phase, "skid_h": h})
            return np.array([x[0], self.hold[1], self.hold[2]]), np.array([self._v0, 0.0, 0.0]), 0.0
        if self.phase == "approach":
            v_app = self.v_oei + self.v_margin
            if v > v_app + 2.0:  # decelerate level first
                v_ref = max(v - 2.0 / _kv(sim), v_app)
                vz = 0.0
            else:  # at (or accelerating to) the approach speed, descending
                v_ref = v_app
                vz = -float(np.clip(self.sink_k * h, 0.5, self.sink_max))
            if h <= self.flare_h:
                self.phase = "flare"
                self.log.append({"t": t, "phase": "flare", "skid_h": h})
            return np.array([x[0], self.hold[1], x[2]]), np.array([v_ref, 0.0, vz]), 0.0
        if self.phase == "flare":
            v_ref = max(v - self.flare_decel / _kv(sim), 0.0)
            if v > self.flare_slow:  # still fast: hold (or regain) flare_h while decelerating
                vz = float(np.clip(0.5 * (self.flare_h - h), -0.5, 1.0))
            else:
                vz = -float(np.clip(0.2 * (h - self.cushion_h) + 0.3, 0.3, 1.0))
            if (h <= self.cushion_h and v <= self.flare_slow) or (v < 2.0 and h < self.flare_h):
                self.phase = "cushion"
                sim.set_cushion(True)
                self.log.append({"t": t, "phase": "cushion", "skid_h": h})
            return np.array([x[0], self.hold[1], x[2]]), np.array([v_ref, 0.0, vz]), 0.0
        if self.phase == "cushion":
            if sim.dyn.aero(x, sim.u)[3].on_ground:
                self.phase = "landed"
                sim.set_cushion(False)
                sim.collective_hold = sim.p.theta_min
                self.log.append({"t": t, "phase": "landed"})
            v_ref = max(v - self.flare_decel / _kv(sim), 0.0)
            return np.array([x[0], self.hold[1], x[2]]), np.array([v_ref, 0.0, -0.3]), 0.0
        return np.array([x[0], x[1], x[2]]), np.zeros(3), 0.0


# ------------------------------------------------------------------------------------------------
@dataclass
class AutorotationLanding:
    """Both engines out: autorotation (vertical or with forward speed), flare and cushion landing."""
    sim: object
    mode: str = "forward"  # "forward" or "vertical"
    v_glide_kt: float = 65.0  # set from min_descent_speed()
    flare_agl: float = 40.0  # m (ESTIMADO; best of the flare sweep, docs/helicoptero/flare_varredura_*.json)
    cushion_agl: float = 4.0  # m (ESTIMADO); vertical mode starts the cushion higher
    level_agl: float = 6.0  # m (ESTIMADO): stop the flare at the latest here
    level_speed: float = 8.0  # m/s (ESTIMADO): flare ends when the ground speed is this low
    flare_decel: float = 4.0  # m/s^2 max deceleration commanded in the flare (ESTIMADO)
    flare_tilt: float = 30.0  # deg, max nose-up attitude in the flare (ESTIMADO, sweep)
    level_decel: float = 1.5  # m/s^2 kept while levelling the attitude (ESTIMADO)
    flare_k: float = 2.0  # m/s: sink-rate error for full flare deceleration (ESTIMADO)
    flare_sink_k: float = 0.2  # 1/s: sink-rate target = k * height above level_agl, 1.5..6 m/s (ESTIMADO)
    cushion_agl_vertical: float = 15.0
    level_tilt: float = 15.0  # deg, attitude limit after the flare (ESTIMADO, sweep)
    cushion_tilt: float = 15.0  # deg, attitude limit while cushioning (ESTIMADO, sweep)
    cushion_sink: float = 0.3  # m/s, sink rate the collective holds in the cushion (ESTIMADO, sweep)
    nr_flare: float = 1.0  # NR reference in glide/flare; 1.04 was tried and overspeeds the rotor on entry (115 %)
    phase: str = "cruise"
    hold: Optional[np.ndarray] = None
    _v: float = 0.0
    _last_t: float = 0.0
    log: List[dict] = field(default_factory=list)

    def __call__(self, t: float, x: np.ndarray):
        sim = self.sim
        dt = max(t - self._last_t, 0.0)
        self._last_t = t
        agl = x[2] - sim.dyn.terrain.height(x[0], x[1])
        if self.hold is None:
            self.hold = x[0:3].copy()
            self._v = float(x[3])
        if self.phase == "cruise":
            if _sadpf_action(sim, t) == "autorrotacao":
                self.phase = "glide"
                sim.set_autorotation(True)
                sim.nr_ref = self.nr_flare
                self.log.append({"t": t, "phase": "glide"})
            v = self._v
            return np.array([x[0], self.hold[1], self.hold[2]]), np.array([v, 0.0, 0.0]), 0.0
        if self.phase == "glide":
            if self.mode == "forward":
                self._v += float(np.clip(self.v_glide_kt * KT - self._v, -1.5 * dt, 1.5 * dt))
                if agl <= self.flare_agl:
                    self.phase = "flare"
                    sim.g.max_tilt_deg = self.flare_tilt
                    self.log.append({"t": t, "phase": "flare", "agl": agl})
                # height is free in autorotation: the attitude follows the speed command only
                return np.array([x[0], self.hold[1], x[2]]), np.array([self._v, 0.0, float(x[5])]), 0.0
            if agl <= self.cushion_agl_vertical:
                self.phase = "cushion"
                sim.set_cushion(True)
                self.log.append({"t": t, "phase": "cushion", "agl": agl})
            return np.array([self.hold[0], self.hold[1], x[2]]), np.zeros(3), 0.0
        if self.phase == "flare":
            # aft cyclic (deceleration) modulated on the sink rate as a pilot does: harder while sinking faster
            # than a height-scheduled target, eased off when it balloons. The NR loop raises the collective as
            # the flare speeds the rotor up, which arrests the sink.
            vz_tgt = -float(np.clip(self.flare_sink_k * (agl - self.level_agl), 1.5, 6.0))
            decel = self.flare_decel * float(np.clip((vz_tgt - x[5]) / self.flare_k + 0.5, 0.15, 1.0))
            self._v = max(float(x[3]) - decel / _kv(sim), 0.0)  # velocity reference that yields `decel`
            if x[3] < self.level_speed or agl <= self.level_agl:
                self.phase = "level"
                sim.g.max_tilt_deg = self.level_tilt
                self.log.append({"t": t, "phase": "level", "agl": agl})
            return np.array([x[0], self.hold[1], x[2]]), np.array([self._v, 0.0, float(x[5])]), 0.0
        if self.phase == "level":
            # attitude back towards level, gentle deceleration, still in autorotation
            self._v = max(float(x[3]) - self.level_decel / _kv(sim), 0.0)
            if agl <= self.cushion_agl or (x[5] < -3.0 and agl < 15.0):
                self.phase = "cushion"
                sim.set_cushion(True)
                sim.g.max_tilt_deg = self.cushion_tilt
                self.log.append({"t": t, "phase": "cushion", "agl": agl})
            return np.array([x[0], self.hold[1], x[2]]), np.array([self._v, 0.0, float(x[5])]), 0.0
        if self.phase == "cushion":
            if sim.dyn.aero(x, sim.u)[3].on_ground:
                self.phase = "landed"
                sim.set_cushion(False)
                sim.collective_hold = sim.p.theta_min
                self.log.append({"t": t, "phase": "landed"})
            self._v = max(float(x[3]) - self.level_decel / _kv(sim), 0.0)
            return (np.array([x[0], self.hold[1], x[2] - self.cushion_sink]),
                    np.array([self._v, 0.0, -self.cushion_sink]), 0.0)
        return np.array([x[0], x[1], x[2]]), np.zeros(3), 0.0


# Touchdown outcome by vertical speed (Passo 4 approval). Thresholds:
#  * 14 CFR 29.725(a): limit drop test from "at least 8 inches" -> sqrt(2 g 0.2032 m) = 2.0 m/s (DERIVADO; a
#    regulatory minimum, the real gear may be designed for more)
#  * 14 CFR 29.727: reserve energy drop "1.5 times that specified in 29.725(a)" -> 2.45 m/s (DERIVADO)
#  * 14 CFR 29.562(b)(1): seat dynamic test, "change in downward velocity of not less than 30 feet per second"
#    -> 9.14 m/s (DERIVADO). Treating a touchdown above it as non-survivable is ESTIMADO: 29.562 is a minimum
#    design condition for seats, not a measured survivability limit.
TD_LIMIT_MS = math.sqrt(2 * G0 * 8 * 0.0254)
TD_RESERVE_MS = math.sqrt(1.5) * TD_LIMIT_MS
TD_SEAT_MS = 30 * FT
TOUCHDOWN_CLASSES = (
    ("pouso", "POUSO", "dentro da queda-limite do trem (29.725(a), 8 in)"),
    ("pouso_duro", "POUSO DURO", "acima da queda-limite, dentro da reserva de energia do trem (29.727): inspeção obrigatória"),
    ("dano_provavel", "POUSO DURO — DANO ESTRUTURAL PROVÁVEL", "acima da reserva de energia do trem (29.727)"),
    ("impacto", "IMPACTO — NÃO SOBREVIVÍVEL", "acima de 30 ft/s, o pulso vertical dos ensaios de assento (29.562(b)(1)); classificação ESTIMADA"),
)


def classify_touchdown(sink_ms: float) -> Dict[str, object]:
    i = 0 if sink_ms <= TD_LIMIT_MS else 1 if sink_ms <= TD_RESERVE_MS else 2 if sink_ms <= TD_SEAT_MS else 3
    code, label, why = TOUCHDOWN_CLASSES[i]
    return {"class": code, "label": label, "why": why, "level": i, "sink_ms": float(sink_ms),
            "thresholds_ms": {"limit_29_725": TD_LIMIT_MS, "reserve_29_727": TD_RESERVE_MS, "seat_29_562": TD_SEAT_MS}}


def find_touchdown(t, ground, vz, agl_skid, t_from: Optional[float] = None, min_air_s: float = 1.0,
                   bounce_gap_s: float = 2.0, bounce_height_m: float = 1.5) -> Optional[Dict[str, object]]:
    """The touchdown that ends the flight (correction of the aval do Passo 4).

    Contacts are the air -> ground transitions. The landing is the LAST contact of the record (the one before the
    aircraft stops) together with the bounces just before it: a previous contact belongs to the same landing when
    the aircraft was back in the air for at most `bounce_gap_s` and never higher than `bounce_height_m`. The first
    contact of that landing must come after `t_from` (e.g. the engine failure) and after the aircraft has been
    airborne for at least `min_air_s` (so a skid bounce at lift-off is never taken as the touchdown). The sink
    rate is the largest among the contacts of the landing. Returns None when there is no such touchdown."""
    t, ground, vz, agl_skid = (np.asarray(a) for a in (t, ground, vz, agl_skid))
    ground = ground.astype(bool)
    n = len(t)
    if n < 2:
        return None
    dt = float(np.median(np.diff(t)))
    n_air = max(int(round(min_air_s / dt)), 1)
    contacts = [k for k in range(1, n) if ground[k] and not ground[k - 1]]

    def valid_first(k):
        return (t_from is None or t[k] >= t_from) and k >= n_air and not ground[k - n_air:k].any()

    if not contacts:
        return None
    seq = [contacts[-1]]
    for j in reversed(contacts[:-1]):
        nxt = seq[0]
        lift = j + int(np.argmax(~ground[j:nxt])) if (~ground[j:nxt]).any() else nxt  # leaves the ground again
        gap_s = t[nxt] - t[lift]
        if gap_s <= bounce_gap_s and float(np.max(agl_skid[lift:nxt], initial=0.0)) <= bounce_height_m:
            seq.insert(0, j)
        else:
            break
    while seq and not valid_first(seq[0]):  # e.g. the lift-off bounce: not part of the landing
        seq.pop(0)
    if not seq:
        return None
    sinks = [float(-min(vz[max(k - 2, 0):k + 1])) for k in seq]
    i = int(np.argmax(sinks))
    return {"k": seq[0], "k_last": seq[-1], "contacts": seq, "t": float(t[seq[0]]), "t_last": float(t[seq[-1]]),
            "sink_ms": max(sinks), "k_max": seq[i], "n_bounces": len(seq) - 1}


def evaluate_landing(tel, t_from: Optional[float] = None, min_air_s: float = 1.0) -> Dict[str, float]:
    """The touchdown that ends the flight (see find_touchdown)."""
    t, vz, vx, vy = (tel.column(k) for k in ("t", "vz", "vx", "vy"))
    ground, agl = tel.column("on_ground") > 0, tel.column("agl")
    agl_skid = agl - (float(np.median(agl[ground])) if ground.any() else 0.0)  # height of the skids above ground
    td = find_touchdown(t, ground, vz, agl_skid, t_from=t_from, min_air_s=min_air_s)
    if td is None:
        return {"landed": False}
    k, sink = td["k"], td["sink_ms"]
    return {"landed": True, "touchdown_t": td["t"], "touchdown_sink_ms": sink, "outcome": classify_touchdown(sink),
            "n_bounces": td["n_bounces"], "final_contact_t": td["t_last"],
            "touchdown_pitch_up_deg": float(-tel.column("pitch_deg")[k]),
            "touchdown_ground_speed_ms": float(np.hypot(vx[k], vy[k])),
            "max_tilt_after_deg": float(np.max(np.hypot(tel.column("roll_deg"), tel.column("pitch_deg"))[k:]))}


# ------------------------------------------------------------------------------------------------
@dataclass
class PointToPoint:
    """Take-off, climb, cruise, approach on a straight glide path, hover and vertical landing."""
    sim: object
    dest: tuple  # (x, y) of the landing point
    cruise_alt: float = 300.0  # m above the take-off surface
    cruise_kt: float = 100.0
    approach_deg: float = 8.0
    hover_height: float = 10.0
    takeoff_height: float = 15.0
    decel: float = 0.8  # m/s^2
    max_roc: float = 4.0
    phase: str = "takeoff"
    _v: float = 0.0
    _last_t: float = 0.0
    log: List[dict] = field(default_factory=list)

    def __call__(self, t: float, x: np.ndarray):
        sim = self.sim
        dt = max(t - self._last_t, 0.0)
        self._last_t = t
        dest = np.asarray(self.dest, float)
        ground_h = sim.dyn.terrain.height(x[0], x[1])
        if self.phase == "takeoff":
            if x[2] >= self.takeoff_height - 0.5:
                self.phase = "enroute"
                self.origin = x[0:2].copy()
                self.log.append({"t": t, "phase": "enroute"})
            return np.array([x[0], x[1], self.takeoff_height]), np.array([0.0, 0.0, 1.5]), 0.0
        d_vec = dest - x[0:2]
        d = float(np.linalg.norm(d_vec))
        u = d_vec / max(d, 1e-6)
        yaw = math.atan2(u[1], u[0]) if d > 30.0 else getattr(self, "_yaw", 0.0)
        self._yaw = yaw
        if self.phase == "enroute":
            v_max = min(self.cruise_kt * KT, math.sqrt(2 * self.decel * max(d - 1.0, 0.0)))
            self._v = min(self._v + 1.0 * dt, v_max)
            z_app = self.hover_height + max(d - 30.0, 0.0) * math.tan(math.radians(self.approach_deg))
            z_ref = min(self.cruise_alt, z_app)
            vz_ref = -self._v * math.tan(math.radians(self.approach_deg)) if z_app < self.cruise_alt else 0.0
            if d < 3.0 and self._v < 1.0:
                self.phase = "final"
                self.log.append({"t": t, "phase": "final"})
            p_ref = np.array([x[0] + u[0] * min(d, 5.0), x[1] + u[1] * min(d, 5.0), z_ref])
            if d < 15.0:
                p_ref[0:2] = dest
            return p_ref, np.array([u[0] * self._v, u[1] * self._v, vz_ref]), yaw
        if self.phase == "final":
            h_skid = x[2] - ground_h - sim.p.cg_h
            vz = -1.0 if h_skid > 3.0 else -0.3
            if sim.dyn.aero(x, sim.u)[3].on_ground:
                self.phase = "landed"
                sim.collective_hold = sim.p.theta_min
                self.log.append({"t": t, "phase": "landed"})
            return np.array([dest[0], dest[1], x[2] + vz]), np.array([0.0, 0.0, vz]), yaw
        return np.array([x[0], x[1], x[2]]), np.zeros(3), yaw
