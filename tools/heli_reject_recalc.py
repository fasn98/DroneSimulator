"""
Recalculation of everything that depends on the reject-branch touchdown, after the correction of the touchdown
detection (aval do Passo 4). Run once per reject procedure:

    python -m tools.heli_reject_recalc v1 [A B C D E]   # procedimento de abortar v1 (sem amortecimento)
    python -m tools.heli_reject_recalc v2 [A B C D E]   # procedimento de abortar v2 (com amortecimento)

  A. Category A maximum mass, 5 conditions: criterion 29.60 and literal 29.59(c) (duct gain G = 1,26) and
     29.60 with G = 1,15 (the lower end of the range).
  B. Maximum take-off mass at the rescue site, TDP 12 m and 17 m.
  C. Mission (cheap): fuel and radius of the three profiles with the masses of A and B.
  D. Failure sweeps around the TDP: elevated heliport at the old and the new Category A masses (sea level ISA and
     1 500 m ISA+25); rescue site at the return take-off masses of C.
  E. Branch prediction: the 130 cases of the Passo 3 sweep (same masses and failure heights), ground truth and
     prediction with the current evaluator; wall time measured with the outer loop in sequence.
Resumable per part; writes docs/helicoptero/recalculo_abortar_<v>.json. ~2-3 h per version on 2 cores.
"""

from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter import HeliParams, calibrate_drag_area  # noqa: E402
from src.physics.helicopter.mission import (FUEL_CAPACITY_AUX_KG, PROFILES, catA_fuel_and_radius,  # noqa: E402
                                            loading_for)
from src.physics.helicopter.rotor import fuel_flow_params  # noqa: E402
from src.physics.helicopter.scenarios import (RESCUE_FAIL_HEIGHTS, CatAConfig, cat_a_max_mass,  # noqa: E402
                                              cat_a_run, failure_height_sweep, rescue_site_config,
                                              rescue_site_max_mass, rescue_site_takeoff)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
CONDS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
NAMES = ["nível do mar, ISA", "nível do mar, ISA+20", "1.000 m, ISA+20", "1.500 m, ISA+25", "1.500 m, ISA+25, proa 8 m/s"]
LO = 1900.0  # kg, lower end of the bisections (below the UTI zero-fuel mass)
VERSION = "v1"


def _name(c):
    return NAMES[CONDS.index(tuple(c))]


def _strip(r):
    return {k: v for k, v in r.items() if k not in ("config",)}


def _catA(args):
    c, criterion, g = args
    cfg = CatAConfig(elevation_m=c[0], delta_t=c[1], headwind_ms=c[2], criterion=criterion,
                     params=None if g == 1.26 else {"tr_duct_gain": g}, reject_procedure=VERSION)
    r = cat_a_max_mass(cfg, lo=LO)
    return {"condition": _name(c), "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], "criterion": criterion,
            "duct_gain": g, "mass_kg": r["mass_kg"], "limited_by": r["limited_by"], "log": r["log"]}


def _site(args):
    c, tdp = args
    r = rescue_site_max_mass(*c, tdp_height_m=tdp, lo=LO, reject_procedure=VERSION)
    return {"condition": _name(c), "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], "tdp_height_m": tdp,
            "mass_kg": r["mass_kg"], "limited_by": r["limited_by"], "log": r["log"]}


def _sweep_elev(args):
    c, mass, which = args
    cfg = CatAConfig(elevation_m=c[0], delta_t=c[1], headwind_ms=c[2], reject_procedure=VERSION)
    return {"condition": _name(c), "mass_kg": mass, "which": which, "rows": failure_height_sweep(mass, cfg)}


def _sweep_site(args):
    c, mass, which = args
    r = rescue_site_takeoff(mass, *c, reject_procedure=VERSION)
    return {"condition": _name(c), "which": which, **r}


def _truth(args):
    kind, cond_name, mass, cfg, rel = args
    out = {}
    for act in ("abortar", "prosseguir"):
        r = cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=rel, force_action=act)
        out[act] = {"safe": bool(r["safe"]), "reason": r["reason"], "touchdown_sink_ms": r.get("touchdown_sink_ms")}
    return out


def _advised(args):
    kind, cond_name, mass, cfg, rel = args
    r = cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=rel, follow_advisory=True)
    a = r["advisory"] or {}
    return {"kind": kind, "condition": cond_name, "mass_kg": mass, "fail_rel_tdp_m": rel,
            "flown": r["branch"], "safe": bool(r["safe"]), "reason": r["reason"],
            "procedure_action": a.get("procedure_action"), "alert": a.get("alert"), "advise": a.get("advise"),
            "predicted": a.get("predicted"), "wall_time_s": a.get("wall_time_s"), "mode": a.get("mode")}


def mission(res):
    """Fuel and radius of the three profiles with the recomputed masses (same logic as heli_step3_report)."""
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    p.ff_idle_kgh, p.sfc_marginal = fuel_flow_params(p)
    m126 = {(r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in res["catA"]
            if r["criterion"] == "elevado_29_60" and r["duct_gain"] == 1.26}
    m115 = {(r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in res["catA"]
            if r["criterion"] == "elevado_29_60" and r["duct_gain"] == 1.15}
    site = {(r["elevation_m"], r["delta_t"], r["headwind_ms"], r["tdp_height_m"]): r["mass_kg"] for r in res["site"]}
    rows = []
    for c in CONDS:
        row = {"condition": _name(c), "cat_a_mass_kg": m126[c], "cat_a_mass_g115_kg": m115[c],
               "site_limit_tdp12_kg": site.get((*c, 12.0)), "site_limit_tdp17_kg": site.get((*c, 17.0)),
               "profiles": {}}
        for prof in PROFILES:
            lim = row["site_limit_tdp12_kg"] if prof == "resgate" else None
            r = catA_fuel_and_radius(m126[c], prof, replace(p), site_limit_kg=lim)
            lo = catA_fuel_and_radius(m115[c], prof, replace(p), site_limit_kg=lim)
            aux = catA_fuel_and_radius(m126[c], prof, replace(p), FUEL_CAPACITY_AUX_KG, site_limit_kg=lim)
            d = {k: r[k] for k in ("fuel_kg", "fuel_limited_by", "radius_nm", "takeoff_mass_kg", "return_takeoff_mass_kg",
                                   "return_takeoff_mass_max_kg", "flight_time_min", "reserve_kg", "site_limit_kg",
                                   "site_fuel_max_kg", "site_min_radius_full_fuel_nm")}
            d.update(radius_nm_g115=lo["radius_nm"], fuel_kg_g115=lo["fuel_kg"],
                     aux_tank={k: aux[k] for k in ("fuel_kg", "fuel_limited_by", "radius_nm", "takeoff_mass_kg")})
            if lim is not None and row["site_limit_tdp17_kg"] is not None:
                s17 = catA_fuel_and_radius(m126[c], prof, replace(p), site_limit_kg=row["site_limit_tdp17_kg"])
                d["tdp17"] = {k: s17[k] for k in ("radius_nm", "site_fuel_max_kg", "site_min_radius_full_fuel_nm")}
            row["profiles"][prof] = d
        rows.append(row)
    return rows


def advisory_jobs():
    """The 130 cases of the Passo 3 sweep (same masses and failure heights)."""
    rows = json.load(open(os.path.join(OUT, "passo3c_resultados.json"), encoding="utf-8"))["advisory_sweep"]["rows"]
    jobs = []
    for r in rows:
        if r["kind"] == "heliponto elevado":
            cfg = CatAConfig() if r["condition"].startswith("nivel") else CatAConfig(elevation_m=1500.0, delta_t=25.0)
        else:
            c = next(c for c in CONDS if _name(c) == r["condition"])
            cfg = rescue_site_config(*c)
        jobs.append((r["kind"], r["condition"], r["mass_kg"], replace(cfg, reject_procedure=VERSION),
                     r["fail_rel_tdp_m"]))
    return jobs


def main():
    global VERSION
    VERSION = sys.argv[1]
    parts = sys.argv[2:] or list("ABCDE")
    path = os.path.join(OUT, f"recalculo_abortar_{VERSION}.json")
    res = json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}
    res.update(version=VERSION, label={"v1": "procedimento de abortar v1 (sem amortecimento)",
                                       "v2": "procedimento de abortar v2 (com amortecimento)"}[VERSION])

    def save():
        with open(path, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1, default=float)

    t0 = time.time()
    if "A" in parts:
        jobs = [(c, crit, 1.26) for c in CONDS for crit in ("elevado_29_60", "literal_29_59c")] + \
               [(c, "elevado_29_60", 1.15) for c in CONDS]
        with Pool(2) as pool:
            res["catA"] = pool.map(_catA, jobs, chunksize=1)
        save()
        print("A", round(time.time() - t0), flush=True)
    if "B" in parts:
        with Pool(2) as pool:
            res["site"] = pool.map(_site, [(c, t) for t in (12.0, 17.0) for c in CONDS], chunksize=1)
        save()
        print("B", round(time.time() - t0), flush=True)
    if "C" in parts:
        res["mission"] = mission(res)
        save()
        print("C", round(time.time() - t0), flush=True)
    if "D" in parts:
        b2 = json.load(open(os.path.join(OUT, "passo2b_resultados.json"), encoding="utf-8"))
        old = {(r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in b2["cat_a_max_mass"]
               if r["criterion"] == "elevado_29_60"}
        new = {(r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in res["catA"]
               if r["criterion"] == "elevado_29_60" and r["duct_gain"] == 1.26}
        jobs = [(c, m[c], w) for c in ((0.0, 0.0, 0.0), (1500.0, 25.0, 0.0)) for m, w in ((old, "massa antiga"),
                                                                                       (new, "massa nova"))]
        site_jobs = []
        for row in res["mission"]:
            c = next(c for c in CONDS if _name(c) == row["condition"])
            r = row["profiles"]["resgate"]
            site_jobs += [(c, round(r["return_takeoff_mass_kg"], 1), "raio máximo"),
                          (c, round(r["return_takeoff_mass_max_kg"], 1), "missão mais curta (mais pesada)")]
        with Pool(2) as pool:
            res["sweep_elevated"] = pool.map(_sweep_elev, jobs, chunksize=1)
            res["sweep_site"] = pool.map(_sweep_site, site_jobs, chunksize=1)
        save()
        print("D", round(time.time() - t0), flush=True)
    if "E" in parts:
        jobs = advisory_jobs()
        with Pool(2) as pool:
            truth = pool.map(_truth, jobs, chunksize=2)
        out = [_advised(j) for j in jobs]  # sequential: the predictor uses two worker processes
        rows = []
        for o, tr in zip(out, truth):
            pred = o["predicted"] or {}
            correct = all(pred[k]["safe"] == tr[k]["safe"] for k in tr) if pred else None
            rec = o["procedure_action"]
            proc_safe = tr[rec]["safe"] if rec in tr else None
            rows.append({**o, "truth": tr, "prediction_correct": correct, "procedure_safe": proc_safe,
                         "changed": proc_safe is not None and o["safe"] != proc_safe})
        wall = [r["wall_time_s"] for r in rows if r["wall_time_s"]]
        res["advisory"] = {
            "n_cases": len(rows), "n_prediction_correct": sum(bool(r["prediction_correct"]) for r in rows),
            "n_alerts": sum(bool(r["alert"]) for r in rows), "n_changed": sum(r["changed"] for r in rows),
            "n_unsafe_to_safe": sum(r["changed"] and r["safe"] for r in rows),
            "n_safe_to_unsafe": sum(r["changed"] and not r["safe"] for r in rows),
            "n_wrong_advice": sum(bool(r["alert"]) and not r["truth"][r["advise"]]["safe"] for r in rows),
            "n_missed": sum(r["procedure_safe"] is False and any(v["safe"] for v in r["truth"].values())
                            and not r["alert"] for r in rows),
            "n_no_branch_safe": sum(not any(v["safe"] for v in r["truth"].values()) for r in rows),
            "wall_time_s": {"mean": float(np.mean(wall)), "p95": float(np.percentile(wall, 95)),
                            "max": float(np.max(wall))},
            "note": "tempos medidos em ambiente de desenvolvimento (contêiner Linux, 2 núcleos); não representam "
                    "hardware embarcado", "rows": rows}
        save()
        print("E", round(time.time() - t0), flush=True)
    for r in res.get("catA", []):
        print(r["condition"], r["criterion"], r["duct_gain"], r["mass_kg"], r["limited_by"])
    for r in res.get("site", []):
        print("site", r["condition"], r["tdp_height_m"], r["mass_kg"], r["limited_by"])
    if "advisory" in res:
        print({k: v for k, v in res["advisory"].items() if k != "rows"})


if __name__ == "__main__":
    main()
