"""
Take-off from the HEMS operating site with the patient on board (return leg of the Resgate profile), for the
5 take-off conditions: AEO hover margins and the OEI capability sweep (reject / continue / exposed).

Two masses per condition: the return take-off at the maximum radius, and the heaviest possible return take-off
(shortest mission, almost the whole fuel still on board).

    python -m tools.heli_rescue_site_report      (~20 min on 2 cores)
Writes docs/helicoptero/local_resgate.json and local_resgate.png.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.mission import catA_fuel_and_radius  # noqa: E402
from src.physics.helicopter.scenarios import RESCUE_FAIL_HEIGHTS, rescue_site_takeoff  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
CONDS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
NAMES = ["nível do mar, ISA", "nível do mar, ISA+20", "1.000 m, ISA+20", "1.500 m, ISA+25", "1.500 m, ISA+25, proa 8 m/s"]


def _case(args):
    c, mass, which = args
    r = rescue_site_takeoff(mass, *c)
    return {"condition": NAMES[CONDS.index(c)], "which": which, **r}


def main():
    with open(os.path.join(OUT, "passo2b_resultados.json"), encoding="utf-8") as f:
        b = json.load(f)
    masses = {(r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in b["cat_a_max_mass"]
              if r["criterion"] == "elevado_29_60"}
    jobs = []
    for c in CONDS:
        m = catA_fuel_and_radius(masses[c], "resgate")
        jobs.append((c, round(m["return_takeoff_mass_kg"], 1), "raio máximo"))
        jobs.append((c, round(m["return_takeoff_mass_max_kg"], 1), "missão mais curta (mais pesada)"))
    with Pool(2) as pool:
        res = pool.map(_case, jobs)
    with open(os.path.join(OUT, "local_resgate.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    # state per failure height: 0 both safe, 1 only reject, 2 only continue, 3 exposed (neither)
    cmap = ListedColormap(["#1baf7a", "#2a78d6", "#eda100", "#e34948"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 3.8), dpi=150, sharey=True)
    for ax, which in zip(axes, ("raio máximo", "missão mais curta (mais pesada)")):
        rows = [r for r in res if r["which"] == which]
        grid = np.array([[0 if (s["reject"]["safe"] and s["continue"]["safe"]) else 1 if s["reject"]["safe"]
                          else 2 if s["continue"]["safe"] else 3 for s in r["sweep"]] for r in rows])
        ax.imshow(grid, cmap=cmap, vmin=0, vmax=3, aspect="auto")
        ax.set_xticks(range(len(RESCUE_FAIL_HEIGHTS)), [f"{h:g}" for h in RESCUE_FAIL_HEIGHTS])
        ax.set_yticks(range(len(rows)), [r["condition"] for r in rows], fontsize=7.5)
        ax.set_xlabel("altura dos esquis na falha (m); TDP a 12 m")
        ax.set_title(f"{which}: {rows[0]['mass_kg']:.0f} kg", fontsize=9, loc="left")
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                ax.text(j, i, ["A/P", "A", "P", "×"][grid[i, j]], ha="center", va="center", fontsize=7,
                        color="white")
    fig.suptitle("Decolagem do local de resgate com o paciente: A = abortar seguro, P = prosseguir seguro, "
                 "× = exposto (nenhum seguro)", fontsize=9.5)
    fig.text(0.01, 0.01, "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui "
             "dados do fabricante.", fontsize=6, color="#52514e")
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(os.path.join(OUT, "local_resgate.png"))
    for r in res:
        print(r["condition"], r["which"], r["mass_kg"], round(r["aeo_margin_ige_kw"]), round(r["aeo_margin_oge_kw"]),
              r["exposure"])


if __name__ == "__main__":
    main()
