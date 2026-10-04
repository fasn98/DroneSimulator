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


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "AB"
    path = os.path.join(OUT, "sensibilidade_antitorque.json")
    res = json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}
    if "A" in what:
        res["fin_share"] = [fin_case(s) for s in SHARES]
    if "B" in what:
        res["duct_gain"] = [duct_case(g) for g in GAINS]
        with Pool(2) as pool:
            res["duct_gain_cat_a"] = pool.map(mass_case, [(g, c) for g in GAINS for c in CONDS])
    with open(path, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    print(json.dumps(res, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
