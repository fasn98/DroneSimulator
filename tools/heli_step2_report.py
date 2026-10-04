"""
Passo 2 report of the "Helicóptero UTI" template: runs the scenarios and writes
docs/helicoptero/passo2_resultados.json and the figures used in docs/helicoptero-uti.md.

    python -m tools.heli_step2_report          (about 10 min on 2 cores)
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.scenarios import (CatAConfig, autorotation, cat_a_max_mass, cat_a_run,  # noqa: E402
                                              rescue, transfer)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
CAT_A_CONFIGS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]


def _clean(d):
    if isinstance(d, dict):
        return {k: _clean(v) for k, v in d.items() if k not in ("telemetry", "sim", "config")}
    if isinstance(d, list):
        return [_clean(v) for v in d]
    if isinstance(d, (np.floating, np.integer, np.bool_)):
        return d.item()
    if isinstance(d, float):
        return round(d, 3)
    return d


def _max_mass(c):
    cfg = CatAConfig(elevation_m=c[0], delta_t=c[1], headwind_ms=c[2])
    r = cat_a_max_mass(cfg, tol=25.0)
    return {"elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], **_clean(r)}


def main():
    os.makedirs(OUT, exist_ok=True)
    res = {}
    with Pool(2) as pool:
        res["cat_a_max_mass"] = pool.map(_max_mass, CAT_A_CONFIGS)
    hh = CatAConfig(elevation_m=1500.0, delta_t=25.0)
    m_lim = next(r["mass_kg"] for r in res["cat_a_max_mass"] if r["elevation_m"] == 1500.0 and r["headwind_ms"] == 0)
    demo = {}
    for label, m in (("no_limite", m_lim), ("acima_do_limite", 2700.0)):
        demo[label] = {b: cat_a_run(m, b, hh) for b in ("reject", "continue")}
    res["cat_a_demo_1500m_isa25"] = {k: {b: _clean(v) for b, v in d.items()} for k, d in demo.items()}
    auto = {m: autorotation(m, height=300.0) for m in ("forward", "vertical")}
    res["autorotation_300m"] = {m: _clean(v) for m, v in auto.items()}
    resc = rescue()
    res["rescue"] = _clean(resc)
    tr = transfer(distance_m=20_000.0)
    res["transfer_20km"] = _clean(tr)
    with open(os.path.join(OUT, "passo2_resultados.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    for m, lab in (("forward", "com velocidade à frente"), ("vertical", "vertical")):
        tel = auto[m]["telemetry"]
        ax[0].plot(tel.column("t"), tel.column("agl"), label=lab)
        ax[1].plot(tel.column("t"), -tel.column("vz"), label=lab)
    ax[0].set_ylabel("altura (m)"), ax[1].set_ylabel("razão de descida (m/s)")
    for a in ax:
        a.set_xlabel("tempo (s)"), a.grid(alpha=.3), a.legend()
    fig.suptitle("Autorrotação após falha dupla a 300 m (modelo conceitual)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "autorrotacao.png"), dpi=120)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={"width_ratios": [1, 2]})
    for label, sty in (("no_limite", "-"), ("acima_do_limite", "--")):
        r = demo[label]["reject"]
        tel = r["telemetry"]
        ax[0].plot(tel.column("t"), tel.column("z") - r["sim"].p.cg_h, sty, color="tab:blue",
                   label=f"abortar — {r['mass_kg']:.0f} kg")
        r = demo[label]["continue"]
        tel = r["telemetry"]
        ax[1].plot(tel.column("x"), tel.column("z") - r["sim"].p.cg_h, sty, color="tab:red",
                   label=f"prosseguir — {r['mass_kg']:.0f} kg ({'seguro' if r['safe'] else 'inseguro'})")
    ax[0].axhline(hh.tdp_height_m, color="gray", ls="-.", label="TDP")
    ax[0].set_xlabel("tempo (s)"), ax[0].set_ylabel("altura dos esquis acima do deck (m)")
    ax[0].set_title("Falha antes do TDP: abortar")
    ax[1].plot([-hh.deck_half_size_m, hh.deck_half_size_m], [0, 0], color="k", lw=3, label="deck (20 m)")
    ax[1].axhline(15 * 0.3048, color="gray", ls=":", label="15 ft acima do deck")
    ax[1].axhline(-hh.deck_height_m, color="saddlebrown", lw=2, label="rua (30 m abaixo)")
    ax[1].set_xlim(-20, 160), ax[1].set_xlabel("distância (m)")
    ax[1].set_title("Falha após o TDP: prosseguir")
    for a_ in ax:
        a_.grid(alpha=.3), a_.legend(fontsize=8)
    fig.suptitle("Categoria A em heliponto elevado, 1.500 m ISA+25, sem vento (modelo conceitual)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "categoria_a.png"), dpi=120)
    print(json.dumps({k: v for k, v in res.items() if k == "cat_a_max_mass"}, ensure_ascii=False, indent=1)[:3000])


if __name__ == "__main__":
    main()
