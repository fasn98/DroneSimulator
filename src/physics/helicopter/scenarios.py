"""
Scenarios of the "Helicóptero UTI" template (Passo 2).

Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.

    autorotation(mode)            dual engine failure, autorotation, flare and landing ("forward" or "vertical")
    compare_autorotation()        both modes side by side (descent rates for the HUD)
    cat_a_run(mass, branch)       Category A vertical take-off from an elevated heliport, engine failure just
                                  before the TDP (reject), just after it (continue) or none
    cat_a_max_mass(...)           largest mass at which both branches are safe for the configured
                                  heliport elevation, ISA deviation and headwind
    transfer(...)                 inter-hospital transfer (take-off, cruise, approach, landing)
    rescue(...)                   restricted-area landing in ground effect with crosswind

All results are computed by the physics; nothing is scripted on the failure time. The engine failure is
injected on a condition (height, phase), the SADPF detects it from torque and NR, and the procedure follows
the SADPF recommendation after the pilot reaction time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .isa import HeliAtmosphere
from .model import Terrain, engine_limit_w
from .params import HeliParams
from .rotor import level_flight
from .procedures import (KT, AutorotationLanding, CatATakeoff, PointToPoint, evaluate_cat_a, evaluate_landing,
                         min_descent_speed, vtoss)
from .simulator import EngineFault, HelicopterSimulator


def _atm(elevation_m=0.0, delta_t=0.0, wind_ms=0.0, wind_to_deg=180.0, gust_std=0.0) -> HeliAtmosphere:
    """Wind is given as the direction the air moves TO (math angle, deg): 180 = headwind for flight along +x."""
    return HeliAtmosphere(elevation_m=elevation_m, delta_t=delta_t, wind_mean=wind_ms,
                          wind_direction_deg=wind_to_deg, wind_gust_std=gust_std)


def _sadpf_summary(sim) -> Dict[str, object]:
    s = sim.sadpf
    inj = [e for e in sim.events if e["code"].startswith("engine_failure") and "injet" in e["message"]]
    det = [e for e in s.events if e["level"] == 3]
    out = {"level": s.level, "recommendation": s.recommendation_code, "message": s.recommendation}
    if inj and det:
        out["detection_delay_s"] = det[0]["t"] - inj[0]["t"]
    return out


# ------------------------------------------------------------------------------------------------
def autorotation(mode: str = "forward", height: float = 150.0, seed: int = 0, fail_at: float = 2.0,
                 duration: float = 60.0, atm: Optional[HeliAtmosphere] = None, **proc_kw) -> Dict[str, object]:
    """Both engines fail at `fail_at` s. forward: cruise at the minimum-rate-of-descent speed; vertical: hover."""
    atm = atm or _atm()
    p = HeliParams()
    sim0 = HelicopterSimulator(p, atm, wind=False)
    md = min_descent_speed(sim0.p, atm, height, sim0.p.mass)
    v0 = md["v_md_kt"] * KT if mode == "forward" else 0.0
    sim = HelicopterSimulator(sim0.p, atm, seed=seed, wind=False, start_position=(0.0, 0.0, height),
                              start_velocity=(v0, 0.0, 0.0),
                              engine_faults=[EngineFault(0, fail_at), EngineFault(1, fail_at)], sadpf=True)
    proc = AutorotationLanding(sim, mode=mode, v_glide_kt=md["v_md_kt"], **proc_kw)
    tel = sim.run(duration, proc, 0.1)
    t, vz, agl = tel.column("t"), tel.column("vz"), tel.column("agl")
    glide_t = next((e["t"] for e in proc.log if e["phase"] == "glide"), None)
    end_t = next((e["t"] for e in proc.log if e["phase"] in ("flare", "cushion")), t[-1])
    steady = (t > (glide_t or 0.0) + 0.5 * (end_t - (glide_t or 0.0))) & (t < end_t)
    land = evaluate_landing(tel)
    return {
        "mode": mode, "height_m": height, "v_glide_kt": md["v_md_kt"] if mode == "forward" else 0.0,
        "rod_steady_ms": float(-np.median(vz[steady])) if np.any(steady) else float("nan"),
        "rod_predicted_ms": md["rod_ms"] if mode == "forward" else md["rod_hover_ms_momentum"],
        "min_nr_pct": float(tel.column("nr_pct")[t < land.get("touchdown_t", t[-1])].min()),
        "min_nr_before_cushion_pct": float(tel.column("nr_pct")[(t > (glide_t or 0.0)) & (t < next(
            (e["t"] for e in proc.log if e["phase"] == "cushion"), t[-1]))].min()),
        "max_nr_pct": float(tel.column("nr_pct").max()),
        "sadpf": _sadpf_summary(sim), "phases": proc.log, "landing": land,
        "telemetry": tel, "sim": sim,
    }


def compare_autorotation(height: float = 300.0, seed: int = 0) -> Dict[str, Dict[str, object]]:
    return {m: autorotation(m, height=height, seed=seed) for m in ("forward", "vertical")}


# ------------------------------------------------------------------------------------------------
@dataclass
class CatAConfig:
    elevation_m: float = 0.0  # heliport elevation (MSL)
    delta_t: float = 0.0  # ISA deviation (K)
    headwind_ms: float = 0.0
    deck_height_m: float = 30.0  # deck above the street (ESTIMADO: rooftop of a mid-rise hospital)
    deck_half_size_m: float = 10.0  # 20 m x 20 m deck (ESTIMADO)
    tdp_height_m: float = 12.0  # skid height of the TDP above the deck (ESTIMADO, configurable)
    fail_margin_m: float = 3.0  # pre-TDP failure this far below the TDP, so that it is RECOGNISED before the TDP
    # (detection ~0.7 s + pilot reaction 1 s at 1.5 m/s climb); the decision is taken at recognition
    fail_after_tdp_s: float = 0.5  # post-TDP failure this long after the TDP


def cat_a_run(mass: float, branch: str = "reject", cfg: Optional[CatAConfig] = None, seed: int = 0,
              duration: float = 45.0) -> Dict[str, object]:
    """branch: "reject" (failure at TDP - margin), "continue" (failure after the TDP) or "none" (AEO)."""
    cfg = cfg or CatAConfig()
    atm = _atm(cfg.elevation_m, cfg.delta_t, cfg.headwind_ms)
    p = HeliParams(mass=mass)
    terrain = Terrain(pad_half_size=cfg.deck_half_size_m, pad_height=cfg.deck_height_m)
    sim = HelicopterSimulator(p, atm, seed=seed, wind=cfg.headwind_ms > 0, terrain=terrain, sadpf=True)
    v_toss = vtoss(sim.p, atm, 0.0, mass)
    proc = CatATakeoff(sim, tdp_height=cfg.tdp_height_m, vtoss_kt=v_toss or 60.0)

    def guidance(t, x):
        if not any(sim.dyn.engine_failed):
            if branch == "reject" and proc.phase == "vertical" and \
                    proc.skid_height(x) >= cfg.tdp_height_m - cfg.fail_margin_m:
                sim.fail_engine(0)
            elif branch == "continue" and proc.tdp_t is not None and t >= proc.tdp_t + cfg.fail_after_tdp_s:
                sim.fail_engine(0)
        return proc(t, x)

    tel = sim.run(duration, guidance, 0.1)
    ev = evaluate_cat_a(tel, sim, v_toss, branch if branch != "none" else "continue")
    if branch != "none" and sim.fail_time is None:
        ev["safe"] = False
        ev["reason"] = ("não atingiu o TDP com os dois motores (sem potência para pairar fora do efeito solo)"
                        if proc.tdp_t is None and branch == "continue" or proc.phase == "vertical"
                        else "falha não injetada")
    oei = tel.column("rating") > 0
    t, z = tel.column("t"), tel.column("z")
    if sim.fail_time is not None:
        za = z[t >= sim.fail_time]
        ev["max_height_loss_m"] = float(np.max(np.maximum.accumulate(za) - za))  # largest drop after the failure
        ev["oei30_used_s"] = float(np.sum(tel.column("rating") == 1) * 0.1)
    ev.update(mass_kg=mass, sadpf=_sadpf_summary(sim) if any(sim.dyn.engine_failed) else None,
              phases=proc.log, max_oei_rating=int(tel.column("rating").max()))
    # power margins computed from the physics (HUD): OEI 30 s available vs hover OGE required at the deck,
    # and OEI 2 min available vs required at VTOSS
    rho = atm.density(0.0)
    p_hover = level_flight(sim.p, 0.0, rho, mass).p_engines
    ev["oei30_margin_hover_kw"] = (engine_limit_w(sim.p, atm, 0.0, "OEI30", 1) - p_hover) / 1e3
    if v_toss is not None:
        ev["oei2_margin_vtoss_kw"] = (engine_limit_w(sim.p, atm, 0.0, "OEI2", 1)
                                      - level_flight(sim.p, v_toss * KT, rho, mass).p_engines) / 1e3
    ev["telemetry"], ev["sim"] = tel, sim
    return ev


def cat_a_max_mass(cfg: Optional[CatAConfig] = None, lo: float = 2200.0, hi: Optional[float] = None,
                   tol: float = 25.0, seed: int = 0) -> Dict[str, object]:
    """Bisection on mass: the largest mass at which BOTH the reject and the continue branches are safe.

    Capped at MTOW. Monotonic behaviour in mass is assumed (heavier = less OEI margin), which the bisection
    log lets one check.
    """
    cfg = cfg or CatAConfig()
    hi = hi or HeliParams().mass
    log: List[dict] = []

    def safe(m):
        r = cat_a_run(m, "reject", cfg, seed)
        c = cat_a_run(m, "continue", cfg, seed)
        log.append({"mass_kg": m, "reject": r["safe"], "reject_reason": r["reason"], "continue": c["safe"],
                    "continue_reason": c["reason"]})
        return r["safe"] and c["safe"]

    if safe(hi):
        return {"mass_kg": hi, "limited_by": "MTOW", "log": log, "config": cfg}
    if not safe(lo):
        return {"mass_kg": None, "limited_by": f"inseguro mesmo com {lo:.0f} kg", "log": log, "config": cfg}
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if safe(mid):
            lo = mid
        else:
            hi = mid
    failing = log[-1] if not (log[-1]["reject"] and log[-1]["continue"]) else \
        next(e for e in reversed(log) if not (e["reject"] and e["continue"]))
    which = "abortar" if not failing["reject"] else "prosseguir"
    return {"mass_kg": lo, "limited_by": which, "log": log, "config": cfg}


# ------------------------------------------------------------------------------------------------
def transfer(distance_m: float = 20_000.0, cruise_alt: float = 300.0, cruise_kt: float = 110.0, seed: int = 0,
             wind_ms: float = 5.0, wind_to_deg: float = 200.0, duration: float = 900.0) -> Dict[str, object]:
    """Inter-hospital transfer: heliport to heliport (both at ground level here)."""
    atm = _atm(wind_ms=wind_ms, wind_to_deg=wind_to_deg, gust_std=1.0)
    sim = HelicopterSimulator(atmosphere=atm, seed=seed, sadpf=True)
    proc = PointToPoint(sim, dest=(distance_m, 0.0), cruise_alt=cruise_alt, cruise_kt=cruise_kt)
    tel = sim.run(duration, lambda t, x: proc(t, x) if proc.phase != "landed" else proc(t, x), 0.5)
    land = evaluate_landing(tel)
    fuel = tel.column("fuel_kg")
    landed_t = next((e["t"] for e in proc.log if e["phase"] == "landed"), None)
    return {"landing": land, "phases": proc.log, "flight_time_s": landed_t, "fuel_used_kg": float(fuel[0] - fuel[-1]),
            "final_position_error_m": float(np.hypot(tel.column("x")[-1] - distance_m, tel.column("y")[-1])),
            "telemetry": tel, "sim": sim}


def rescue(crosswind_ms: float = 8.0, seed: int = 0, duration: float = 200.0, approach_deg: float = 12.0,
           distance_m: float = 1500.0) -> Dict[str, object]:
    """Restricted-area landing (e.g. road accident site): steep approach, hover in ground effect, crosswind
    from the left of the track (air moving towards -y)."""
    atm = _atm(wind_ms=crosswind_ms, wind_to_deg=-90.0, gust_std=1.5)
    sim = HelicopterSimulator(atmosphere=atm, seed=seed, sadpf=True, start_position=(0.0, 0.0, 150.0),
                              start_velocity=(40.0, 0.0, 0.0))
    proc = PointToPoint(sim, dest=(distance_m, 0.0), cruise_alt=150.0, cruise_kt=80.0, approach_deg=approach_deg,
                        hover_height=4.0, phase="enroute")
    proc._v = 40.0
    tel = sim.run(duration, proc, 0.1)
    land = evaluate_landing(tel)
    final = next((e["t"] for e in proc.log if e["phase"] == "final"), None)
    t = tel.column("t")
    hov = (t >= (final or t[-1]) - 3.0) & (t <= (final or t[-1]))
    p_oge = level_flight(sim.p, crosswind_ms, atm.density(0.0)).p_engines / 1e3
    return {"landing": land, "phases": proc.log, "power_oge_same_wind_kw": p_oge,
            "k_ge_hover": float(np.mean(tel.column("k_ge")[hov])) if np.any(hov) else None,
            "power_hover_kw": float(np.mean((tel.column("p_eng1_kw") + tel.column("p_eng2_kw"))[hov]))
            if np.any(hov) else None,
            "max_pedal": float(np.max(np.abs(tel.column("pedal")))),
            "max_bank_deg": float(np.max(np.abs(tel.column("roll_deg")))),
            "final_position_error_m": float(np.hypot(tel.column("x")[-1] - distance_m, tel.column("y")[-1])),
            "telemetry": tel, "sim": sim}
