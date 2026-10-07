"""
Passo 3 complements (approval of Passo 3):
  A. maximum take-off mass at the HEMS operating site (rescue site) with TDP 12 m (default) and 17 m (sensitivity),
     in the 5 conditions;
  B. "previsão de ramos" (SADPF advisory): the failure sweeps around the TDP flown again with the "pilot" following
     the advisory, compared with the procedure alone; prediction accuracy against the forced-branch results;
     wall-clock time of the prediction.

    python -m tools.heli_step3c_report [A|B|AB]      (~30 min on 2 cores; resumable per part)
Writes docs/helicoptero/passo3c_resultados.json.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.scenarios import (RESCUE_FAIL_HEIGHTS, CatAConfig, cat_a_run,  # noqa: E402
                                              rescue_site_config, rescue_site_max_mass)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
PATH = os.path.join(OUT, "passo3c_resultados.json")
CONDS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
NAMES = ["nível do mar, ISA", "nível do mar, ISA+20", "1.000 m, ISA+20", "1.500 m, ISA+25", "1.500 m, ISA+25, proa 8 m/s"]
SAFE_KEY = {"abortar": "reject", "prosseguir": "continue"}


def _max_mass(args):
    c, tdp = args
    r = rescue_site_max_mass(*c, tdp_height_m=tdp)
    return {"condition": NAMES[CONDS.index(c)], "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2],
            "tdp_height_m": tdp, "mass_kg": r["mass_kg"], "limited_by": r["limited_by"], "log": r["log"]}


def _truth(args):
    """Ground truth with the current code: both branches forced, full-resolution simulation."""
    kind, cond_name, mass, cfg, rel, _ = args
    return {act: bool(cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=rel, force_action=act)["safe"])
            for act in ("abortar", "prosseguir")}


def _advised(args):
    """One failure flown with the pilot following the advisory; returns outcome and the advisory itself."""
    kind, cond_name, mass, cfg, rel, truth = args
    r = cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=rel, follow_advisory=True)
    a = r["advisory"] or {}
    if truth is None:  # ground truth with the current code: both branches forced, full-resolution simulation
        truth = {act: bool(cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=rel, force_action=act)["safe"])
                 for act in ("abortar", "prosseguir")}
    return {"kind": kind, "condition": cond_name, "mass_kg": mass, "fail_rel_tdp_m": rel, "truth": truth,
            "flown": r["branch"], "safe": r["safe"], "reason": r["reason"],
            "procedure_action": a.get("procedure_action"), "alert": a.get("alert"), "advise": a.get("advise"),
            "predicted": a.get("predicted"), "wall_time_s": a.get("wall_time_s"),
            "parallel_estimate_s": a.get("parallel_estimate_s"), "mode": a.get("mode"),
            "sequential_estimate_s": a.get("sequential_estimate_s")}


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "AB"
    res = json.load(open(PATH, encoding="utf-8")) if os.path.exists(PATH) else {}

    def save():
        with open(PATH, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1, default=float)

    if "A" in what:
        with Pool(2) as pool:
            res["rescue_max_mass"] = pool.map(_max_mass, [(c, t) for t in (12.0, 17.0) for c in CONDS])
        save()

    if "B" in what:
        # baselines: forced-branch results already computed (procedure outcome = branch the SADPF recommended)
        b2 = json.load(open(os.path.join(OUT, "passo2b_resultados.json"), encoding="utf-8"))
        lr = json.load(open(os.path.join(OUT, "local_resgate.json"), encoding="utf-8"))
        jobs, base = [], []
        # the forced-branch ground truth does not depend on the predictor: reuse it from a previous run
        # (delete "advisory_sweep" from the JSON to recompute it, e.g. after a change in the model)
        old = {(r["kind"], r["condition"], round(r["mass_kg"], 1), r["fail_rel_tdp_m"]): r["truth"]
               for r in res.get("advisory_sweep", {}).get("rows", []) if "truth" in r}
        catA = {"nivel_do_mar_2980kg": (2980.0, CatAConfig()),
                **{k: (float(k.split("_")[-1][:-2]), CatAConfig(elevation_m=1500.0, delta_t=25.0))
                   for k in b2["failure_height_sweep"] if k.startswith("1500m")}}
        for name, rows in b2["failure_height_sweep"].items():
            mass, cfg = catA[name]
            for r in rows:
                jobs.append(("heliponto elevado", name, mass, cfg, r["fail_rel_tdp_m"],
                             old.get(("heliponto elevado", name, round(mass, 1), r["fail_rel_tdp_m"]))))
                base.append(r)
        for case in lr:
            c = (case["elevation_m"], case["delta_t"], case["headwind_ms"])
            cfg = rescue_site_config(*c)
            for r in case["sweep"]:
                rel = r["fail_skid_h_m"] - cfg.tdp_height_m
                jobs.append(("local de resgate", case["condition"], case["mass_kg"], cfg, rel,
                             old.get(("local de resgate", case["condition"], round(case["mass_kg"], 1), rel))))
                base.append(r)
        # sequential outer loop: the predictor itself runs the two branches in two worker processes (Passo 3
        # approval), and the wall time must be measured on an otherwise idle machine
        missing = [i for i, j in enumerate(jobs) if j[5] is None]
        if missing:  # truth first, two cases at a time (its timing does not matter)
            with Pool(2) as pool:
                for i, tr in zip(missing, pool.map(_truth, [jobs[i] for i in missing])):
                    jobs[i] = (*jobs[i][:5], tr)
        prev_truth = {(r["kind"], r["condition"], round(r["mass_kg"], 1), r["fail_rel_tdp_m"]): r["truth"]
                      for r in res.get("advisory_sweep_previous_truth", [])}
        out = [_advised(j) for j in jobs]
        rows = []
        for o, b in zip(out, base):
            rec = o["procedure_action"] or b.get("recommended")
            truth = o["truth"]
            proc_safe = truth[rec] if rec in truth else None
            pred = o["predicted"] or {}
            correct = all(pred[k]["safe"] == truth[k] for k in truth) if pred else None
            rows.append({**o, "procedure_safe": proc_safe, "prediction_correct": correct,
                         "changed": (proc_safe is not None and o["safe"] != proc_safe)})
        wall = [r["wall_time_s"] for r in rows if r["wall_time_s"]]
        par = [r["parallel_estimate_s"] for r in rows if r["parallel_estimate_s"]]
        res["advisory_sweep"] = {
            "n_cases": len(rows), "n_alerts": sum(bool(r["alert"]) for r in rows),
            "n_changed": sum(r["changed"] for r in rows),
            "n_unsafe_to_safe": sum(r["changed"] and r["safe"] for r in rows),
            "n_safe_to_unsafe": sum(r["changed"] and not r["safe"] for r in rows),
            "n_prediction_correct": sum(bool(r["prediction_correct"]) for r in rows),
            "n_with_prediction": sum(r["prediction_correct"] is not None for r in rows),
            "n_wrong_advice": sum(bool(r["alert"]) and not r["truth"][r["advise"]] for r in rows),
            "n_missed": sum((r["procedure_safe"] is False) and any(r["truth"].values()) and not r["alert"]
                            for r in rows),
            "n_over_budget": sum((r["wall_time_s"] or 0) > 1.0 for r in rows),
            # ground truth recomputed after a model change, compared with the previous one (same cases)
            "n_truth_changed_vs_previous": sum(
                1 for r in rows if (k := (r["kind"], r["condition"], round(r["mass_kg"], 1), r["fail_rel_tdp_m"]))
                in prev_truth and prev_truth[k] != r["truth"]),
            "truth_changed_cases": [[r["kind"], r["condition"], r["mass_kg"], r["fail_rel_tdp_m"], prev_truth[k], r["truth"]]
                                    for r in rows if (k := (r["kind"], r["condition"], round(r["mass_kg"], 1),
                                                            r["fail_rel_tdp_m"])) in prev_truth and prev_truth[k] != r["truth"]],
            "wall_time_s": {"mean": float(np.mean(wall)), "max": float(np.max(wall)),
                            "p95": float(np.percentile(wall, 95))},
            "parallel_estimate_s": {"mean": float(np.mean(par)), "max": float(np.max(par))},
            "sequential_estimate_s": {"mean": float(np.mean([r["sequential_estimate_s"] for r in rows])),
                                      "max": float(np.max([r["sequential_estimate_s"] for r in rows]))},
            "modes": sorted({r["mode"] for r in rows}),
            "note": "tempos de relógio medidos em ambiente de desenvolvimento (contêiner Linux, 2 núcleos); "
                    "não representam hardware embarcado",
            "cpu": os.cpu_count(), "rows": rows}
        save()
    print(json.dumps({k: v for k, v in res.get("advisory_sweep", {}).items() if k != "rows"}, ensure_ascii=False,
                     indent=1))
    for r in res.get("rescue_max_mass", []):
        print(r["condition"], r["tdp_height_m"], r["mass_kg"], r["limited_by"])


if __name__ == "__main__":
    main()
