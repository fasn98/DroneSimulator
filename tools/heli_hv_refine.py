"""
H-V checks requested at the Passo 2 approval:
  1. sea level, ISA, 2 980 kg: grid resolution and the worst margin among the safe points
     (margin = how close the touchdown came to the safe-landing limits);
  2. 1 500 m ISA+25 at the Category A mass: grid refined 4x (or more) around 10 m / 10 kt, and the clearance of the
     Category A take-off path from the unsafe region.

    python -m tools.heli_hv_refine          (about 20 min on 2 cores; needs docs/helicoptero/passo2b_resultados.json)
Writes docs/helicoptero/hv_refino.json and hv_refino.png.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.procedures import KT  # noqa: E402
from src.physics.helicopter.scenarios import (HV_GS_MAX, HV_SINK_MAX, HV_TILT_MAX, CatAConfig, _atm,  # noqa: E402
                                              cat_a_run, hv_point)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
REF_H = np.arange(6.0, 14.01, 0.5)  # m (coarse grid step 2 m around 10 m -> 0.5 m: 4x)
REF_V = np.arange(0.0, 20.01, 2.5)  # kt (coarse step 10 kt -> 2.5 kt: 4x)


def margin(t):
    """Fraction of the most critical safe-landing limit used at touchdown (1.0 = on the limit)."""
    if not t.get("landed"):
        return float("inf")
    return max((t["touchdown_sink_ms"] or 0) / HV_SINK_MAX, (t["touchdown_ground_speed_ms"] or 0) / HV_GS_MAX,
               (t["max_tilt_after_deg"] or 0) / HV_TILT_MAX)


def _pt(args):
    h, v, mass, elev, dt = args
    r = hv_point(float(h), float(v), mass=mass, atm=_atm(elev, dt))
    r["margin_used"] = min(margin(t) for t in r["tries"])
    r["tries"] = [{k: t[k] for k in ("technique", "safe", "touchdown_sink_ms", "touchdown_ground_speed_ms",
                                     "max_tilt_after_deg")} | {"margin_used": margin(t)} for t in r["tries"]]
    return r


def main():
    with open(os.path.join(OUT, "passo2b_resultados.json"), encoding="utf-8") as f:
        b = json.load(f)
    m_hh = next(r["mass_kg"] for r in b["cat_a_max_mass"] if r["criterion"] == "elevado_29_60"
                and r["elevation_m"] == 1500.0 and r["headwind_ms"] == 0.0)
    res = {}
    # 1) sea level: worst margin among the coarse-grid points (recomputed with the margin kept)
    sl_pts = [p for p in b["hv"]["nível do mar, ISA, 2.980 kg"]["points"]]
    hs = sorted({p["height_m"] for p in sl_pts})
    vs = sorted({p["speed_kt"] for p in sl_pts})
    with Pool(2) as pool:
        sl = pool.map(_pt, [(h, v, 2980.0, 0.0, 0.0) for h in hs for v in vs])
    worst = max(sl, key=lambda r: r["margin_used"])
    res["sea_level"] = {"grid_heights_m": hs, "grid_speeds_kt": vs, "height_steps_m": np.diff(hs).tolist(),
                        "speed_step_kt": 10.0, "n_points": len(sl), "n_unsafe": sum(not r["safe"] for r in sl),
                        "worst_point": {k: worst[k] for k in ("height_m", "speed_kt", "technique", "touchdown_sink_ms",
                                                              "touchdown_ground_speed_ms", "max_tilt_after_deg",
                                                              "margin_used")},
                        "points": [{k: r[k] for k in ("height_m", "speed_kt", "safe", "technique", "margin_used")}
                                   for r in sl]}
    # 2) refined grid around 10 m / 10 kt at 1 500 m ISA+25, Category A mass
    with Pool(2) as pool:
        ref = pool.map(_pt, [(h, v, m_hh, 1500.0, 25.0) for h in REF_H for v in REF_V])
    r = cat_a_run(m_hh, "none", CatAConfig(elevation_m=1500.0, delta_t=25.0), duration=40.0)
    tel = r["telemetry"]
    ph = tel.column("agl") - r["sim"].p.cg_h
    pv = np.hypot(tel.column("vx"), tel.column("vy")) / KT
    unsafe = [(p["speed_kt"], p["height_m"]) for p in ref if not p["safe"]]
    box = (pv <= REF_V[-1] + 1e-9) & (ph >= REF_H[0]) & (ph <= REF_H[-1])
    clear = None
    if unsafe and np.any(box):
        dv, dh = 2.5, 0.5
        best = None
        for i in np.where(box)[0]:
            for (uv, uh) in unsafe:
                d = float(np.hypot((pv[i] - uv) / dv, (ph[i] - uh) / dh))
                if best is None or d < best[0]:
                    best = (d, float(pv[i]), float(ph[i]), uv, uh)
        clear = {"min_distance_grid_steps": best[0], "path_point_kt_m": best[1:3], "unsafe_point_kt_m": best[3:5]}
    res["hot_high_refined"] = {
        "mass_kg": m_hh, "heights_m": REF_H.tolist(), "speeds_kt": REF_V.tolist(), "height_step_m": 0.5,
        "speed_step_kt": 2.5, "refinement_vs_coarse": "4x em altura (2 m -> 0,5 m) e 4x em velocidade (10 -> 2,5 kt)",
        "n_points": len(ref), "unsafe_points_kt_m": unsafe,
        "path_in_box": [(round(a, 2), round(c, 2)) for a, c in zip(pv[box], ph[box])],
        "clearance": clear,
        "points": [{k: p[k] for k in ("height_m", "speed_kt", "safe", "technique", "margin_used")} for p in ref]}
    with open(os.path.join(OUT, "hv_refino.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
    m = np.array([[next(p["margin_used"] for p in sl if p["height_m"] == h and p["speed_kt"] == v) for v in vs]
                  for h in hs])
    im = ax[0].pcolormesh(vs, hs, np.minimum(m, 2.0), shading="nearest", cmap="RdYlGn_r", vmin=0, vmax=2.0)
    ax[0].scatter([worst["speed_kt"]], [worst["height_m"]], marker="*", s=160, color="k", label="pior margem")
    ax[0].set_title("Nível do mar, ISA, 2.980 kg: fração do limite usada no toque")
    ax[0].set_xlabel("velocidade (kt)"), ax[0].set_ylabel("altura dos esquis (m)"), ax[0].legend(fontsize=8)
    fig.colorbar(im, ax=ax[0], label="1,0 = no limite de pouso seguro")
    for p in ref:
        ax[1].scatter(p["speed_kt"], p["height_m"], s=30, marker="o" if p["safe"] else "X",
                      color="tab:green" if p["safe"] else "tab:red")
    ax[1].plot(pv, ph, color="tab:blue", lw=2, label="trajetória Cat A AEO")
    ax[1].set_xlim(-1, 21), ax[1].set_ylim(5.5, 14.5)
    ax[1].set_title(f"1.500 m ISA+25, {m_hh:.0f} kg: grade refinada (2,5 kt × 0,5 m)")
    ax[1].set_xlabel("velocidade horizontal (kt)"), ax[1].legend(fontsize=8), ax[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "hv_refino.png"), dpi=120)
    print(json.dumps({"sea_level_worst": res["sea_level"]["worst_point"], "sl_unsafe": res["sea_level"]["n_unsafe"],
                      "refined_unsafe": unsafe, "clearance": clear}, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
