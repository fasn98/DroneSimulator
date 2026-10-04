"""
Passo 3 report: UTI interior, mass and balance, fuel and radius of action for the Category A take-off conditions.

    python -m tools.heli_step3_report     (~2 min; reads the Category A masses from docs/helicoptero/passo2b_resultados.json)
Writes docs/helicoptero/passo3_resultados.json, raio_acao.png, cg_envelope.png, interior_uti.png and the mass table
of docs/helicoptero-uti.md (between the MASSA markers).
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import replace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter import HeliParams, calibrate_drag_area  # noqa: E402
from src.physics.helicopter.isa import HeliAtmosphere  # noqa: E402
from src.physics.helicopter.mission import (EMPTY_MASS, FUEL_CAPACITY_AUX_KG, FUEL_CAPACITY_KG,  # noqa: E402
                                            FUEL_STA_MM, Loading, catA_fuel_and_radius, cg_limits)
from src.physics.helicopter.rotor import fuel_flow_params  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "helicoptero")
DOC = os.path.join(ROOT, "docs", "helicoptero-uti.md")
CONDS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
S1, S2 = "#2a78d6", "#eb6834"


def label(c):
    e = "nível do mar" if c[0] == 0 else f"{c[0]:.0f} m".replace("1000", "1.000").replace("1500", "1.500")
    t = "ISA" if c[1] == 0 else f"ISA+{c[1]:.0f}"
    w = f", proa {c[2]:.0f} m/s" if c[2] else ""
    return f"{e}, {t}{w}"


def fmt(x, nd=0):
    s = f"{x:,.{nd}f}".replace(",", "X").replace(".", ",").replace("X", ".")
    return s


def main():
    with open(os.path.join(OUT, "passo2b_resultados.json"), encoding="utf-8") as f:
        b = json.load(f)
    masses = {(r["elevation_m"], r["delta_t"], r["headwind_ms"]): r["mass_kg"] for r in b["cat_a_max_mass"]
              if r["criterion"] == "elevado_29_60"}
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    p.ff_idle_kgh, p.sfc_marginal = fuel_flow_params(p)
    loading = Loading()
    rows, rows_aux = [], []
    for c in CONDS:
        atm = HeliAtmosphere(elevation_m=c[0], delta_t=c[1])  # cruise with no wind
        r = catA_fuel_and_radius(masses[c], atm, loading, replace(p))
        rows.append({"condition": label(c), "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], **r})
        ra = catA_fuel_and_radius(masses[c], atm, loading, replace(p), FUEL_CAPACITY_AUX_KG)
        rows_aux.append({"condition": label(c), "elevation_m": c[0], "delta_t": c[1], "headwind_ms": c[2], **ra})
    res = {"fuel_model": {"ff_idle_kgh_per_engine": p.ff_idle_kgh, "sfc_marginal_kg_kwh": p.sfc_marginal},
           "zero_fuel_mass_kg": loading.zero_fuel_mass,
           "items": [vars(i) for i in loading.items],
           "empty_cg_range_mm": loading.empty_cg_range([0.0, 50.0, FUEL_CAPACITY_KG]),
           "conditions": rows, "conditions_aux_tank": rows_aux}
    with open(os.path.join(OUT, "passo3_resultados.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "text.color": INK, "axes.facecolor": SURF, "figure.facecolor": SURF})
    disclaimer = "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante."

    # radius of action x take-off condition: two series (standard tank, with auxiliary tank)
    fig, ax = plt.subplots(figsize=(10, 4.8), dpi=150)
    y = np.arange(len(rows))[::-1] * 1.0
    hgt = 0.36
    for k, (rr, col, lab) in enumerate(((rows, S1, f"tanque padrão ({FUEL_CAPACITY_KG:.0f} kg)"),
                                        (rows_aux, S2, f"com tanque auxiliar ({FUEL_CAPACITY_AUX_KG:.0f} kg)"))):
        yy = y + (0.2 if k == 0 else -0.2)
        ax.barh(yy, [r["radius_nm"] or 0 for r in rr], height=hgt, color=col, label=lab)
        for yi, r in zip(yy, rr):
            ax.text((r["radius_nm"] or 0) + 2, yi, f"{r['radius_nm']:.0f} NM · {r['fuel_kg']:.0f} kg "
                    f"({r['fuel_limited_by']})", va="center", fontsize=7, color=INK2)
    for yi, r in zip(y, rows):
        ax.text(-3, yi, f"{r['condition']}\nmassa Cat A {r['cat_a_mass_kg']:.0f} kg", va="center", ha="right",
                fontsize=7.5, color=INK)
    ax.set_yticks([])
    ax.set_xlim(0, max(r["radius_nm"] for r in rows_aux) * 1.45)
    ax.set_xlabel("raio de ação, ida e volta, com reserva VFR de 20 min (NM)")
    ax.set_title("Configuração UTI padrão: raio de ação × condição de decolagem Categoria A (14 CFR 29.60)",
                 fontsize=9.5, loc="left")
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s_ in ("top", "right", "left"):
        ax.spines[s_].set_visible(False)
    ax.legend(fontsize=7.5, loc="upper right", frameon=False)
    fig.text(0.01, 0.01, disclaimer, fontsize=6, color=INK2)
    fig.tight_layout(rect=(0.2, 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, "raio_acao.png"))

    # C.G. envelope with take-off and reserve-only states
    fig, ax = plt.subplots(figsize=(7.5, 5), dpi=150)
    ms = np.linspace(1500, 3175, 50)
    fwd = [cg_limits(m)[0] for m in ms]
    aft = [cg_limits(m)[1] for m in ms]
    ax.fill_betweenx(ms, fwd, aft, color=GRID, alpha=0.6, lw=0)
    ax.plot(fwd, ms, color=INK2, lw=1.2)
    ax.plot(aft, ms, color=INK2, lw=1.2)
    ax.text(4200, 3120, "envelope TCDS R.009 (EC135 T3H)", fontsize=7, color=INK2)
    for r in rows:
        a, z = r["cg_takeoff"], r["cg_reserve_only"]
        ax.plot([a["sta_mm"], z["sta_mm"]], [a["mass_kg"], z["mass_kg"]], color=INK2, lw=0.8, ls=":")
    ax.scatter([r["cg_takeoff"]["sta_mm"] for r in rows], [r["cg_takeoff"]["mass_kg"] for r in rows], s=40, color=S1,
               label="decolagem (combustível máximo da condição)", zorder=3)
    ax.scatter([r["cg_takeoff"]["sta_mm"] for r in rows_aux], [r["cg_takeoff"]["mass_kg"] for r in rows_aux], s=40,
               facecolors="none", edgecolors=S1, label="decolagem com tanque auxiliar", zorder=3)
    ax.scatter([r["cg_reserve_only"]["sta_mm"] for r in rows], [r["cg_reserve_only"]["mass_kg"] for r in rows], s=40,
               color=S2, marker="s", label="fim do voo (só a reserva de 20 min)", zorder=3)
    ax.set_xlabel("CG longitudinal (mm atrás do plano de referência, STA)")
    ax.set_ylabel("massa (kg)")
    ax.set_title("CG da configuração UTI nas 5 condições Cat A", fontsize=9.5, loc="left")
    ax.legend(fontsize=7.5, loc="lower right", frameon=False)
    ax.grid(color=GRID, lw=0.6)
    fig.text(0.01, 0.01, disclaimer, fontsize=6, color=INK2)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, "cg_envelope.png"))

    # interior plan view (generic, schematic): numbered markers, legend list beside the plan
    fig = plt.figure(figsize=(12, 4.6), dpi=150)
    ax = fig.add_axes([0.06, 0.14, 0.56, 0.76])
    hull = np.array([[1700, 0], [1900, 650], [2300, 760], [5200, 760], [5600, 450], [5600, -450], [5200, -760],
                     [2300, -760], [1900, -650], [1700, 0]])
    ax.fill(hull[:, 0], hull[:, 1], color=GRID, alpha=0.5, lw=1.0, ec=INK2)
    ax.add_patch(plt.Rectangle((3150, -450), 2000, 300, color="#d6e4f5", ec=S1, lw=1))  # stretcher (left side)
    ax.text(4800, -560, "maca", fontsize=7, color=S1)
    legend_lines = []
    seen = {}
    for n, i in enumerate(loading.items[1:], start=1):
        key = (i.sta_mm, i.bl_mm)
        off = seen.get(key, 0)
        seen[key] = off + 1
        col = S2 if i.group == "tripulação" else (INK if i.group == "paciente" else S1)
        ax.scatter(i.sta_mm, i.bl_mm, s=26, color=col, zorder=3)
        ax.text(i.sta_mm + 30 + 90 * off, i.bl_mm + 40, str(n), fontsize=6.5, color=INK)
        legend_lines.append(f"{n:>2}. {i.name} — {fmt(i.mass_kg, 1)} kg")
    ax.scatter(EMPTY_MASS.sta_mm, EMPTY_MASS.bl_mm, marker="P", s=60, color=INK, zorder=3)
    ax.text(EMPTY_MASS.sta_mm + 30, -110, "CG vazio", fontsize=6.5, color=INK)
    ax.scatter(FUEL_STA_MM, 0, marker="D", s=30, color=INK2, zorder=3)
    ax.text(FUEL_STA_MM - 260, 80, "tanque", fontsize=6.5, color=INK2)
    ax.set_xlim(1600, 5800), ax.set_ylim(-900, 900)
    ax.set_aspect("equal")
    ax.set_xlabel("STA (mm atrás do plano de referência) — frente à esquerda")
    ax.set_yticks([-600, 0, 600], ["600\nesq.", "0", "600\ndir."])
    ax.set_ylabel("BL (mm)")
    ax.set_title("Interior UTI genérico, vista de cima (posições ESTIMADAS)", fontsize=9.5, loc="left")
    fig.text(0.64, 0.9, "\n".join(legend_lines), fontsize=6.6, va="top", color=INK, family="DejaVu Sans")
    fig.text(0.01, 0.01, disclaimer, fontsize=6, color=INK2)
    fig.savefig(os.path.join(OUT, "interior_uti.png"))

    # mass table in the doc
    lines = ["| Item | Massa (kg) | STA (mm) | BL (mm, + dir.) | Status da massa | Fonte / justificativa |",
             "|---|---|---|---|---|---|"]
    for i in loading.items:
        lines.append(f"| {i.name} | {fmt(i.mass_kg, 1)} | {fmt(i.sta_mm)} | {fmt(i.bl_mm)} | **{i.status}** | {i.source} |")
    lines.append(f"| **Massa sem combustível** | **{fmt(loading.zero_fuel_mass, 1)}** | | | | |")
    lines.append(f"| Combustível utilizável, tanque padrão (máx.) | {fmt(FUEL_CAPACITY_KG)} | {fmt(FUEL_STA_MM)} | 0 | "
                 f"**FONTE** (massa) | folheto Airbus; posição do tanque ESTIMADA |")
    text = open(DOC, encoding="utf-8").read()
    a, z = "<!-- MASSA:BEGIN (gerado por tools/heli_step3_report.py) -->", "<!-- MASSA:END -->"
    if a in text:
        i0, i1 = text.index(a) + len(a), text.index(z)
        text = text[:i0] + "\n" + "\n".join(lines) + "\n" + text[i1:]
        open(DOC, "w", encoding="utf-8").write(text)
    print(json.dumps({k: v for k, v in res.items() if k != "items"}, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
