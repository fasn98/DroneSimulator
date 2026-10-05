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
from .procedures import (KT, AutorotationLanding, CatATakeoff, OeiLanding, PointToPoint, evaluate_cat_a,
                         evaluate_landing, min_descent_speed, vtoss)
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
    criterion: str = "elevado_29_60"  # or "literal_29_59c" (see procedures.evaluate_cat_a)
    params: Optional[dict] = None  # HeliParams overrides (sensitivity studies), e.g. {"tr_duct_gain": 1.15}


def cat_a_run(mass: float, branch: str = "reject", cfg: Optional[CatAConfig] = None, seed: int = 0,
              duration: float = 45.0, fail_rel_tdp_m: Optional[float] = None,
              force_action: Optional[str] = None) -> Dict[str, object]:
    """branch: "reject" (failure at TDP - margin), "continue" (failure after the TDP) or "none" (AEO).

    fail_rel_tdp_m: if given, the engine fails when the skids reach TDP + fail_rel_tdp_m (negative = below the
    TDP), whatever the branch; the branch flown is then the SADPF recommendation, or `force_action`
    ("abortar" / "prosseguir") to fly a given branch, and it is judged as that branch.
    """
    cfg = cfg or CatAConfig()
    atm = _atm(cfg.elevation_m, cfg.delta_t, cfg.headwind_ms)
    p = HeliParams(mass=mass, **(cfg.params or {}))
    terrain = Terrain(pad_half_size=cfg.deck_half_size_m, pad_height=cfg.deck_height_m)
    sim = HelicopterSimulator(p, atm, seed=seed, wind=cfg.headwind_ms > 0, terrain=terrain, sadpf=True)
    v_toss = vtoss(sim.p, atm, 0.0, mass)
    proc = CatATakeoff(sim, tdp_height=cfg.tdp_height_m, vtoss_kt=v_toss or 60.0, force_action=force_action)

    def guidance(t, x):
        if not any(sim.dyn.engine_failed):
            if fail_rel_tdp_m is not None:
                if proc.skid_height(x) >= cfg.tdp_height_m + fail_rel_tdp_m:
                    sim.fail_engine(0)
            elif branch == "reject" and proc.phase == "vertical" and \
                    proc.skid_height(x) >= cfg.tdp_height_m - cfg.fail_margin_m:
                sim.fail_engine(0)
            elif branch == "continue" and proc.tdp_t is not None and t >= proc.tdp_t + cfg.fail_after_tdp_s:
                sim.fail_engine(0)
        return proc(t, x)

    tel = sim.run(duration, guidance, 0.1)
    if fail_rel_tdp_m is not None:
        flown = next((e["phase"] for e in proc.log if e["phase"] in ("reject", "continue")), "continue")
        branch = flown
    ev = evaluate_cat_a(tel, sim, v_toss, branch if branch != "none" else "continue", cfg.criterion)
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
              action_skid_h_m=next((e.get("skid_h") for e in proc.log if e["phase"] in ("reject", "continue")), None),
              detect_skid_h_m=(float(np.interp(sim.sadpf.detect_t, t, z)) - sim.p.cg_h)
              if sim.sadpf.detect_t is not None else None,
              fail_skid_h_m=(float(np.interp(sim.fail_time, t, z)) - sim.p.cg_h) if sim.fail_time is not None else None,
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


GAP_FAILURES = (-1.0, -0.5, 0.0)  # m relative to the TDP


def cat_a_max_mass(cfg: Optional[CatAConfig] = None, lo: float = 2200.0, hi: Optional[float] = None,
                   tol: float = 25.0, seed: int = 0) -> Dict[str, object]:
    """Bisection on mass: the largest mass at which BOTH the reject and the continue branches are safe.

    Capped at MTOW. Monotonic behaviour in mass is assumed (heavier = less OEI margin), which the bisection
    log lets one check.
    """
    cfg = cfg or CatAConfig()
    hi = hi or HeliParams().mass  # MTOW (the overrides never change it)
    log: List[dict] = []

    def safe(m):
        # reject: failure recognised before the TDP. Then failures just below and at the TDP, flown as the SADPF
        # recommends: they are recognised around the TDP (detection ~0.7 s), so this covers the gap where the
        # failure happens before the TDP but is recognised after it (continue from a lower height and speed).
        r = cat_a_run(m, "reject", cfg, seed)
        gap = [cat_a_run(m, cfg=cfg, seed=seed, fail_rel_tdp_m=dh) for dh in GAP_FAILURES]
        bad_c = [g for g in gap if not g["safe"] and g["branch"] == "continue"]
        bad_r = [g for g in gap if not g["safe"] and g["branch"] == "reject"]
        log.append({"mass_kg": m, "reject": r["safe"] and not bad_r,
                    "reject_reason": r["reason"] if not r["safe"] else (bad_r[0]["reason"] if bad_r else "ok"),
                    "continue": not bad_c, "continue_reason": bad_c[0]["reason"] if bad_c else "ok",
                    "gap": [{"fail_rel_tdp_m": dh, "branch": g["branch"], "safe": g["safe"], "reason": g["reason"]}
                            for dh, g in zip(GAP_FAILURES, gap)]})
        return log[-1]["reject"] and log[-1]["continue"]

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


def failure_height_sweep(mass: float, cfg: Optional[CatAConfig] = None,
                         rel_heights=tuple(np.arange(-6.0, 6.01, 1.0)), seed: int = 0) -> List[dict]:
    """For each failure height relative to the TDP: what the SADPF recommends, and whether flying the reject
    branch and the continue branch (forced) is safe."""
    cfg = cfg or CatAConfig()
    out = []
    for dh in rel_heights:
        row = {"fail_rel_tdp_m": float(dh)}
        for act, key in (("abortar", "reject"), ("prosseguir", "continue")):
            r = cat_a_run(mass, cfg=cfg, seed=seed, fail_rel_tdp_m=float(dh), force_action=act)
            row[key] = {"safe": r["safe"], "reason": r["reason"],
                        "recommended": (r["sadpf"] or {}).get("recommendation"),
                        "detect_rel_tdp_m": (r["detect_skid_h_m"] - cfg.tdp_height_m)
                        if r.get("detect_skid_h_m") is not None else None,
                        "action_rel_tdp_m": (r["action_skid_h_m"] - cfg.tdp_height_m)
                        if r.get("action_skid_h_m") is not None else None,
                        "touchdown_sink_ms": r.get("touchdown_sink_ms"),
                        "max_height_loss_m": r.get("max_height_loss_m")}
        row["recommended"] = row["reject"]["recommended"]
        out.append(row)
    return out


# ------------------------------------------------------------------------------------------------
HV_SINK_MAX = 2.0  # m/s: free fall from the 8 in drop height of 14 CFR 29.725(a), sqrt(2 g 0.203 m) (DERIVADO)
HV_GS_MAX = 15 * KT  # m/s, run-on ground speed accepted (ESTIMADO, same as the autorotation goal)
HV_TILT_MAX = 15.0  # deg after touchdown (no roll-over) (ESTIMADO)


def v_oei_level(p, atm, mass) -> float:
    """Lowest speed (m/s) at which OEI 30 s power holds level flight out of ground effect (DERIVADO)."""
    rho = atm.density(0.0)
    p_av = engine_limit_w(p, atm, 0.0, "OEI30", 1)
    for v in np.arange(0.0, 60.0, 0.5):
        if level_flight(p, v, rho, mass).p_engines <= p_av:
            return float(v)
    return 30.0


def _hv_try(height_m, speed_kt, p, atm, seed, fail_at, technique):
    v = speed_kt * KT
    sim = HelicopterSimulator(p, atm, seed=seed, wind=False, sadpf=True,
                              start_position=(0.0, 0.0, height_m + p.cg_h), start_velocity=(v, 0.0, 0.0))
    if technique == "frente":
        proc = OeiLanding(sim, v_oei=v_oei_level(sim.p, atm, sim.p.mass))
    else:  # "vertical": no acceleration, slow descent straight down into ground effect, then cushion
        proc = OeiLanding(sim, v_oei=0.0, v_margin=0.0, sink_max=1.0)
    sim.faults = [EngineFault(0, fail_at)]
    tel = sim.run(40.0 + height_m / 2.0, proc, 0.05)
    land = evaluate_landing(tel)
    safe = bool(land.get("landed") and land["touchdown_sink_ms"] <= HV_SINK_MAX
                and land["touchdown_ground_speed_ms"] <= HV_GS_MAX and land["max_tilt_after_deg"] <= HV_TILT_MAX)
    return {"technique": technique, "safe": safe, **{k: land.get(k) for k in (
        "landed", "touchdown_sink_ms", "touchdown_ground_speed_ms", "max_tilt_after_deg")},
        "min_nr_pct": float(tel.column("nr_pct").min())}


def hv_point(height_m: float, speed_kt: float, mass: Optional[float] = None, atm: Optional[HeliAtmosphere] = None,
             seed: int = 0, fail_at: float = 1.0) -> Dict[str, object]:
    """Engine failure at skid height `height_m` and airspeed `speed_kt` in level flight, then an OEI landing
    straight ahead. Two techniques are tried, "frente" (approach at v_oei + margin, flare) and, from 20 kt or
    less, "vertical" (straight slow descent); the point is safe if either gives a safe landing."""
    atm = atm or _atm()
    p = HelicopterSimulator(HeliParams(mass=mass) if mass else HeliParams(), atm, wind=False).p
    tries = [_hv_try(height_m, speed_kt, p, atm, seed, fail_at, "frente")]
    if speed_kt <= 20.0:
        tries.append(_hv_try(height_m, speed_kt, p, atm, seed, fail_at, "vertical"))
    best = next((r for r in tries if r["safe"]), min(tries, key=lambda r: r.get("touchdown_sink_ms") or 99.0))
    return {"height_m": height_m, "speed_kt": speed_kt, **best, "tries": tries}


def hv_boundary(points: List[dict]) -> Dict[float, Optional[tuple]]:
    """Per speed, the unsafe height band (lowest unsafe, highest unsafe) from the grid; None if all safe."""
    out = {}
    for v in sorted({p["speed_kt"] for p in points}):
        bad = sorted(p["height_m"] for p in points if p["speed_kt"] == v and not p["safe"])
        out[v] = (bad[0], bad[-1]) if bad else None
    return out


def path_in_hv(path_v_kt, path_h_m, points: List[dict], mode: str = "inside") -> List[int]:
    """Indices of path samples in the unsafe region of the grid.

    mode "inside": the 4 grid points around the sample are all unsafe; "touches": at least one is unsafe
    (the sample lies in a cell crossed by the H-V boundary, i.e. within one grid step of it).
    """
    vs = np.array(sorted({p["speed_kt"] for p in points}))
    hs = np.array(sorted({p["height_m"] for p in points}))
    safe = {(p["speed_kt"], p["height_m"]): p["safe"] for p in points}
    idx = []
    for i, (v, h) in enumerate(zip(path_v_kt, path_h_m)):
        if h < hs[0] or h > hs[-1] or v > vs[-1]:
            continue
        # bilinear neighbourhood: inside if the 4 surrounding grid points are all unsafe (conservative = not
        # flagged on the boundary) -- report also "touches" (any of the 4 unsafe)
        iv = int(np.clip(np.searchsorted(vs, v) - 1, 0, len(vs) - 2))
        ih = int(np.clip(np.searchsorted(hs, h) - 1, 0, len(hs) - 2))
        corners = [safe[(vs[a], hs[b])] for a in (iv, iv + 1) for b in (ih, ih + 1)]
        if (not any(corners)) if mode == "inside" else (not all(corners)):
            idx.append(i)
    return idx


# ------------------------------------------------------------------------------------------------
RESCUE_FAIL_HEIGHTS = (1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 12.5, 14.0, 17.0, 20.0)  # m skid height (grid of the sweep)


def rescue_site_takeoff(mass: float, elevation_m: float = 0.0, delta_t: float = 0.0, headwind_ms: float = 0.0,
                        tdp_height_m: float = 12.0, heights=RESCUE_FAIL_HEIGHTS, seed: int = 0) -> Dict[str, object]:
    """Take-off from a ground-level HEMS operating site (the return leg of the Resgate profile, patient on board).

    1. AEO power margin in hover in and out of ground effect (take-off rating, site atmosphere).
    2. OEI capability along the same vertical take-off profile as the Category A procedure (TDP above the
       surface): for each failure height, flying the reject branch and the continue branch (forced). Continue is
       judged with 14 CFR 29.59(c) (>= 15 ft above the take-off surface after the failure) and 29.67(a)(1); the
       site is assumed flat and free of obstacles. Failure heights where neither branch is safe form the exposed
       interval; its duration is read on the AEO take-off profile.
    """
    cfg = CatAConfig(elevation_m=elevation_m, delta_t=delta_t, headwind_ms=headwind_ms, deck_height_m=0.0,
                     deck_half_size_m=math.inf, tdp_height_m=tdp_height_m, criterion="literal_29_59c")
    atm = _atm(elevation_m, delta_t, headwind_ms)
    p = HelicopterSimulator(HeliParams(mass=mass), atm, wind=False).p
    rho = atm.density(0.0)
    avail = 2 * engine_limit_w(p, atm, 0.0, "TO", 2) / 1e3
    oge = level_flight(p, 0.0, rho, mass).p_engines / 1e3
    ige = level_flight(p, 0.0, rho, mass, z_hub=1.0 + p.cg_h + p.hub_h).p_engines / 1e3  # skids 1 m up
    rows = []
    for h in heights:
        row = {"fail_skid_h_m": float(h)}
        for act, key in (("abortar", "reject"), ("prosseguir", "continue")):
            r = cat_a_run(mass, cfg=cfg, seed=seed, fail_rel_tdp_m=float(h) - tdp_height_m, force_action=act)
            row[key] = {"safe": r["safe"], "reason": r["reason"]}
        row["recommended"] = (r["sadpf"] or {}).get("recommendation")
        row["exposed"] = not (row["reject"]["safe"] or row["continue"]["safe"])
        rows.append(row)
    aeo = cat_a_run(mass, "none", cfg, seed=seed, duration=30.0)
    tel = aeo["telemetry"]
    t, hs = tel.column("t"), tel.column("z") - p.cg_h
    exp_h = [r["fail_skid_h_m"] for r in rows if r["exposed"]]
    exposure = None
    if exp_h:
        lo, hi = min(exp_h), max(exp_h)
        i_lo = int(np.argmax(hs >= lo - 1e-6))
        i_hi = int(np.argmax(hs >= hi - 1e-6))
        exposure = {"from_skid_h_m": lo, "to_skid_h_m": hi, "duration_s": float(t[i_hi] - t[i_lo]),
                    "note": f"resolução da grade de altura: {np.diff(sorted(heights)).max():.1f} m"}
    return {"mass_kg": mass, "elevation_m": elevation_m, "delta_t": delta_t, "headwind_ms": headwind_ms,
            "aeo_available_kw": avail, "hover_oge_kw": oge, "hover_ige_kw": ige,
            "aeo_margin_oge_kw": avail - oge, "aeo_margin_ige_kw": avail - ige,
            "aeo_reaches_tdp": aeo["phases"] and any(e["phase"] == "tdp" for e in aeo["phases"]),
            "sweep": rows, "exposure": exposure,
            "always_safe": not exp_h and all(r["reject"]["safe"] or r["continue"]["safe"] for r in rows)}


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
    fuel_end = float(np.interp(landed_t, tel.column("t"), fuel)) if landed_t else float(fuel[-1])  # at touchdown
    return {"landing": land, "phases": proc.log, "flight_time_s": landed_t, "fuel_used_kg": float(fuel[0] - fuel_end),
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
