"""
Sensitivity of the anti-torque model (approval of Passo 2, decisions 1 and 3).

  A. fin share of the anti-torque at 136 kt from 20 % to 60 % (fin incidence solved for each share, drag area
     recalibrated by the same criterion each time): calibrated f, max level speed at AEO take-off power,
     Fenestron power at 136 kt;
  B. duct thrust gain G from 1.10 to 1.26: Fenestron power in hover, total hover OGE power (MTOW, ISA SL),
     AEO margin, and Category A masses in the 5 conditions (criterion 14 CFR 29.60).

    python -m tools.heli_antitorque_sensitivity [A|B|AB]     (B: ~2 h on 2 cores)
Writes docs/helicoptero/sensibilidade_antitorque.json and .png.
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

import numpy as np
from scipy.optimize import brentq

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter import HeliParams, calibrate_drag_area, level_flight  # noqa: E402
from src.physics.helicopter.scenarios import CatAConfig, cat_a_max_mass  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
KT = 0.514444
SHARES = [0.2, 0.3, 0.4, 0.5, 0.6]
GAINS = [1.10, 1.15, 1.20, 1.26]
CONDS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]


def share(p, v_kt=136.0):
    t = level_flight(p, v_kt * KT, 1.225)
    return t.f_fin / (t.f_fin + t.t_tail)


def v_at_to(p):
    return brentq(lambda x: level_flight(p, x * KT, 1.225).p_engines / 1e3 - p.gearbox_limit_kw("TO"), 90, 200)


def fin_case(s):
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    for _ in range(6):  # fixed point: incidence for the share, then recalibrate f
        a0 = brentq(lambda a: share(replace(p, fin_alpha0=a)) - s, 0.0, math.radians(25.0))
        p = replace(p, fin_alpha0=a0)
        p.f_drag = calibrate_drag_area(p)
    t136 = level_flight(p, 136 * KT, 1.225)
    return {"fin_share_136kt": s, "fin_incidence_deg": math.degrees(p.fin_alpha0), "f_drag_m2": p.f_drag,
            "v_max_at_aeo_to_kt": v_at_to(p), "tail_power_136kt_kw": t136.p_tail / 1e3,
            "fin_cl_136kt": p.fin_a * p.fin_alpha0}


def duct_case(g):
    p = HeliParams(tr_duct_gain=g)
    p.f_drag = calibrate_drag_area(p)
    h = level_flight(p, 0.0, 1.225)
    return {"duct_gain": g, "tail_power_hover_kw": h.p_tail / 1e3, "hover_oge_total_kw": h.p_engines / 1e3,
            "aeo_margin_hover_kw": p.gearbox_limit_kw("TO") - h.p_engines / 1e3, "f_drag_m2": p.f_drag}


def mass_case(args):
    g, c = args
    cfg = CatAConfig(elevation_m=c[0], delta_t=c[1], headwind_ms=c[2], params={"tr_duct_gain": g})
    r = cat_a_max_mass(cfg, tol=25.0)
    return {"duct_gain": g, "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], "mass_kg": r["mass_kg"],
            "limited_by": r["limited_by"]}


def plot(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink2, grid = "#52514e", "#e4e3df"
    cols = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.4), dpi=150)
    fs = res["fin_share"]
    x = [100 * r["fin_share_136kt"] for r in fs]
    ax[0].plot(x, [r["tail_power_136kt_kw"] for r in fs], color=cols[0], lw=2, marker="o")
    for xi, r in zip(x, fs):
        ax[0].annotate(f"f = {r['f_drag_m2']:.2f} m²\nV_máx = {r['v_max_at_aeo_to_kt']:.1f} kt", (xi, r["tail_power_136kt_kw"]),
                       xytext=(0, 10), textcoords="offset points", ha="center", fontsize=6.5, color=ink2)
    ax[0].set_ylabel("potência do Fenestron a 136 kt (kW)")
    ax[0].set_ylim(0, 50)
    ax[0].set_xlabel("parcela do antitorque assumida pela deriva a 136 kt (%)")
    ax[0].set_title("Deriva: sensibilidade (f recalibrada em cada caso)", fontsize=9.5, loc="left")
    ax[0].grid(color=grid)
    conds = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
    names = ["nível do mar, ISA", "nível do mar, ISA+20", "1.000 m, ISA+20", "1.500 m, ISA+25", "1.500 m, ISA+25, proa 8 m/s"]
    m = {(r["duct_gain"], r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in res["duct_gain_cat_a"]}
    gs = sorted({r["duct_gain"] for r in res["duct_gain_cat_a"]})
    for c, n, col in zip(conds, names, cols):
        ax[1].plot(gs, [m[(g, *c)] for g in gs], color=col, lw=2, marker="o", label=n)
    ax[1].set_xlabel("ganho de empuxo do duto G (1,26 = duto ideal)")
    ax[1].set_ylabel("massa máx. Categoria A, modo 29.60 (kg)")
    ax[1].set_title("Duto do Fenestron: massas Cat A (bisseção de 25 kg)", fontsize=9.5, loc="left")
    ax[1].grid(color=grid), ax[1].legend(fontsize=7, frameon=False, loc="lower right")
    fig.text(0.01, 0.01, "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.",
             fontsize=6, color=ink2)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, "sensibilidade_antitorque.png"))


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "AB"
    path = os.path.join(OUT, "sensibilidade_antitorque.json")
    res = json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}
    if what == "P":
        plot(res)
        return
    if "A" in what:
        res["fin_share"] = [fin_case(s) for s in SHARES]
    def save():
        with open(path, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1, default=float)

    if "B" in what:
        res["duct_gain"] = [duct_case(g) for g in GAINS]
        done = {(r["duct_gain"], r["elevation_m"], r["delta_t"], r["headwind_ms"])
                for r in res.get("duct_gain_cat_a", [])}
        res.setdefault("duct_gain_cat_a", [])
        todo = [(g, c) for g in GAINS for c in CONDS if (g, *c) not in done]
        save()
        with Pool(2) as pool:  # resumable: every finished case is saved at once
            for r in pool.imap_unordered(mass_case, todo):
                res["duct_gain_cat_a"].append(r)
                save()
    save()
    print(json.dumps(res, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
