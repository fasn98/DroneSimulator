"""
Passo 2 adjustments (2b): Category A masses under both clearance criteria, failure-height sweep around the TDP
and the height-velocity (H-V) diagram with the Category A take-off paths. Writes
docs/helicoptero/passo2b_resultados.json, falha_tdp.png and hv_diagrama.png.

    python -m tools.heli_step2b_report        (about 40 min on 2 cores)
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.procedures import KT  # noqa: E402
from src.physics.helicopter.scenarios import (CatAConfig, _atm, cat_a_max_mass, cat_a_run,  # noqa: E402
                                              failure_height_sweep, hv_boundary, hv_point, path_in_hv)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
CONFIGS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
CRITERIA = ("elevado_29_60", "literal_29_59c")
HV_HEIGHTS = [2.0, 5.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 45.0, 60.0, 90.0]
HV_SPEEDS = [0.0, 10.0, 20.0, 30.0, 40.0, 60.0]
FAIL_RELS = [-6.0, -5.0, -4.0, -3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def _clean(d):
    if isinstance(d, dict):
        return {k: _clean(v) for k, v in d.items() if k not in ("telemetry", "sim", "config")}
    if isinstance(d, (list, tuple)):
        return [_clean(v) for v in d]
    if isinstance(d, (np.floating, np.integer, np.bool_)):
        return d.item()
    if isinstance(d, float):
        return round(d, 3)
    return d


def _max_mass(args):
    c, crit = args
    cfg = CatAConfig(elevation_m=c[0], delta_t=c[1], headwind_ms=c[2], criterion=crit)
    r = cat_a_max_mass(cfg, tol=25.0)
    return {"criterion": crit, "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], **_clean(r)}


def _hv(args):
    cond, h, v = args
    return {"condition": cond["name"], **hv_point(h, v, mass=cond["mass"], atm=_atm(cond["elev"], cond["dt"]))}


def _fail_sweep(args):
    name, mass, cfg = args
    return name, failure_height_sweep(mass, cfg, FAIL_RELS)


def _path(args):
    """AEO Category A take-off path: (airspeed kt, skid height above the surface below m)."""
    name, mass, cfg = args
    r = cat_a_run(mass, "none", cfg, duration=40.0)
    tel = r["telemetry"]
    h = tel.column("agl") - r["sim"].p.cg_h
    v_h = np.hypot(tel.column("vx") + cfg.headwind_ms, tel.column("vy"))  # horizontal airspeed (H-V axis)
    return name, (v_h / KT).tolist(), h.tolist()


def main():
    os.makedirs(OUT, exist_ok=True)
    only_hv = len(sys.argv) > 1 and sys.argv[1] == "hv"  # reuse the masses and sweeps already in the JSON
    if only_hv:
        with open(os.path.join(OUT, "passo2b_resultados.json"), encoding="utf-8") as f:
            res = json.load(f)
        mm = res["cat_a_max_mass"]
    else:
        res = {}
        with Pool(2) as pool:
            mm = pool.map(_max_mass, [(c, k) for k in CRITERIA for c in CONFIGS])
        res["cat_a_max_mass"] = mm
    m_hh = next(r["mass_kg"] for r in mm if r["criterion"] == "elevado_29_60" and r["elevation_m"] == 1500.0
                and r["headwind_ms"] == 0.0)

    sl = CatAConfig()
    hh = CatAConfig(elevation_m=1500.0, delta_t=25.0)
    if only_hv:
        sweeps = res["failure_height_sweep"]
    else:
        with Pool(2) as pool:
            sweeps = dict(pool.map(_fail_sweep, [("nivel_do_mar_2980kg", 2980.0, sl),
                                                 (f"1500m_isa25_{m_hh:.0f}kg", m_hh, hh)]))
        res["failure_height_sweep"] = _clean(sweeps)

    conds = [{"name": "nível do mar, ISA, 2.980 kg", "mass": 2980.0, "elev": 0.0, "dt": 0.0},
             {"name": f"1.500 m, ISA+25, {m_hh:.0f} kg", "mass": m_hh, "elev": 1500.0, "dt": 25.0}]
    with Pool(2) as pool:
        hv = pool.map(_hv, [(c, h, v) for c in conds for h in HV_HEIGHTS for v in HV_SPEEDS])
    ground = replace(sl, deck_height_m=0.0)
    paths_args = [("heliponto no solo, nível do mar, 2.980 kg", 2980.0, ground, conds[0]["name"]),
                  ("heliponto elevado 30 m, nível do mar, 2.980 kg", 2980.0, sl, conds[0]["name"]),
                  (f"heliponto elevado 30 m, 1.500 m ISA+25, {m_hh:.0f} kg", m_hh, hh, conds[1]["name"])]
    with Pool(2) as pool:
        paths = pool.map(_path, [a[:3] for a in paths_args])
    res["hv"] = {}
    for c in conds:
        pts = [p for p in hv if p["condition"] == c["name"]]
        res["hv"][c["name"]] = {"points": _clean(pts), "unsafe_band_by_speed": {
            str(k): v for k, v in hv_boundary(pts).items()}}
    res["catA_paths_vs_hv"] = []
    for (name, v, h), a in zip(paths, paths_args):
        pts = [p for p in hv if p["condition"] == a[3]]
        inside = path_in_hv(v, h, pts)
        touch = path_in_hv(v, h, pts, "touches")
        res["catA_paths_vs_hv"].append({"path": name, "hv_condition": a[3], "samples": len(v),
                                        "samples_inside_unsafe": len(inside), "outside_hv": len(inside) == 0,
                                        "samples_in_boundary_cells": len(touch),
                                        "boundary_cell_points": [(round(v[i], 1), round(h[i], 1)) for i in touch[:20]]})
    with open(os.path.join(OUT, "passo2b_resultados.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    # failure height sweep
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2), sharey=True)
    for ax, (name, rows) in zip(axes, sweeps.items()):
        for yi, key, lab in ((1, "reject", "abortar"), (0, "continue", "prosseguir")):
            for r in rows:
                ok = r[key]["safe"]
                ax.scatter(r["fail_rel_tdp_m"], yi, s=150, marker="o" if ok else "X",
                           color="tab:green" if ok else "tab:red")
        for r in rows:
            rec = r["recommended"]
            ax.annotate({"abortar": "A", "prosseguir": "P"}.get(rec, "?"), (r["fail_rel_tdp_m"], 0.5), ha="center",
                        va="center", fontsize=9, color="tab:blue")
        ax.axvline(0, color="gray", ls="--")
        ax.set_yticks([0, 0.5, 1], ["prosseguir", "SADPF\nrecomenda", "abortar"])
        ax.set_xlabel("altura da falha em relação ao TDP (m)")
        ax.set_title(name.replace("_", " "))
        ax.set_ylim(-0.5, 1.5), ax.grid(alpha=.3)
    axes[0].legend(handles=[Line2D([], [], marker="o", ls="", color="tab:green", label="seguro"),
                            Line2D([], [], marker="X", ls="", color="tab:red", label="inseguro")], loc="lower left")
    fig.suptitle("Categoria A, heliponto elevado: falha em várias alturas em torno do TDP (A = abortar, P = prosseguir)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "falha_tdp.png"), dpi=120)

    # H-V diagrams with the take-off paths
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for ax, c in zip(axes, conds):
        pts = [p for p in hv if p["condition"] == c["name"]]
        for p in pts:
            ax.scatter(p["speed_kt"], p["height_m"], s=60, marker="o" if p["safe"] else "X",
                       color="tab:green" if p["safe"] else "tab:red")
        for (name, v, h), a in zip(paths, paths_args):
            if a[3] == c["name"]:
                ax.plot(v, h, lw=1.8, label=name)
        ax.set_title(c["name"]), ax.set_xlabel("velocidade (kt)"), ax.grid(alpha=.3)
        ax.set_xlim(-3, 65), ax.set_ylim(0, 95)
        ax.legend(fontsize=7, loc="upper right")
    axes[0].set_ylabel("altura dos esquis acima da superfície (m)")
    fig.suptitle("Diagrama altura-velocidade OEI (●pouso seguro, ✕ inseguro) e trajetórias Cat A AEO (modelo conceitual)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "hv_diagrama.png"), dpi=120)
    print(json.dumps({"cat_a_max_mass": [(r["criterion"], r["elevation_m"], r["delta_t"], r["headwind_ms"],
                                          r["mass_kg"], r["limited_by"]) for r in mm],
                      "paths": res["catA_paths_vs_hv"],
                      "hv_bands": {k: v["unsafe_band_by_speed"] for k, v in res["hv"].items()}},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
