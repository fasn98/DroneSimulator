#!/usr/bin/env python3
"""
Generate the Twin v2 validation figures and numbers used in the thesis (Ch. 3 and 7).

    python tools/twin_validation_report.py            # writes docs/twin_v2/
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.physics import (Body, TwinSimulator, hover_at, hover_report, load_body, load_vehicle,  # noqa: E402
                         lunar_delta_v, max_hover_mass)

OUT = ROOT / "docs" / "twin_v2"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"  # categorical slots 1-3 (validated all-pairs)


def style(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    ax.figure.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=12, color=INK, pad=12)
    ax.set_xlabel(xlabel, color=INK2)
    ax.set_ylabel(ylabel, color=INK2)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2)


def feasibility_chart(msh_body: Body) -> dict:
    base = load_vehicle("mars_hexacopter_tcc")
    diam = np.linspace(0.4, 2.0, 81)
    masses = []
    for d in diam:
        v = copy.deepcopy(base)
        for r in v.rotors:
            r.radius = d / 2
        masses.append(max_hover_mass(v, msh_body))
    masses = np.array(masses)
    tcc = load_vehicle("tcc_mars_70kg_original")
    tcc_max = max_hover_mass(tcc, msh_body)

    fig, ax = plt.subplots(figsize=(8, 4.6), dpi=150)
    style(ax, "Massa máxima que 6 rotores sustentam em Marte (ρ = 0,015 kg/m³, Mach de ponta 0,8)",
          "Diâmetro do rotor (m)", "Massa máxima em voo pairado (kg)")
    ax.plot(diam, masses, color=S1, linewidth=2)
    ax.scatter([1.28], [31.2], s=64, color=S3, edgecolor=SURFACE, linewidth=2, zorder=3)
    ax.annotate("NASA Mars Science Helicopter\n31,2 kg · rotores 1,28 m", (1.28, 31.2), xytext=(1.36, 14),
                color=INK, fontsize=9, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax.scatter([0.6], [70.0], s=64, color=S2, edgecolor=SURFACE, linewidth=2, zorder=3)
    ax.annotate(f"TCC Cap. 3: 70 kg · rotores 0,6 m\nlimite físico ≈ {tcc_max:.1f} kg".replace(".", ","), (0.6, 70.0),
                xytext=(0.72, 62), color=INK, fontsize=9, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax.set_xlim(0.4, 2.0)
    ax.set_ylim(0, 80)
    fig.tight_layout()
    fig.savefig(OUT / "feasibility_mars.png")
    plt.close(fig)
    i = int(np.argmin(np.abs(masses - 70.0)))
    return {"tcc_70kg_max_hover_mass_kg": tcc_max, "rotor_diameter_needed_for_70kg_m": float(diam[i])}


def flights() -> dict:
    cases = [
        ("Hexacóptero TCC (base MSH) em Marte", "mars_hexacopter_tcc", "mars", S1),
        ("Drone original do TCC (70 kg) em Marte", "tcc_mars_70kg_original", "mars", S2),
        ("Hopper lunar do TCC na Lua", "lunar_hopper_tcc", "moon", S3),
    ]
    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=150)
    style(ax, "Mesma missão (subir a 10 m e pairar) simulada com física real", "Tempo (s)", "Altitude (m)")
    results = {}
    for label, veh, env, color in cases:
        v, b = load_vehicle(veh), load_body(env)
        sim = TwinSimulator(v, b, seed=7)
        tel = sim.run(45.0, hover_at([0, 0, 10]))
        t, z = tel.column("t"), tel.column("z")
        ax.plot(t, z, color=color, linewidth=2, label=label)
        late = t > 25
        results[veh] = {
            "body": env,
            "max_altitude_m": float(z.max()),
            "hover_error_max_m": float(np.abs(z[late] - 10).max()) if z.max() > 5 else None,
            "mean_power_w": float(tel.column("power_w")[late].mean()),
            "battery_used_wh": float(v.battery_wh - tel.column("battery_wh")[-1]),
            "propellant_used_kg": float(v.propellant_mass - tel.column("propellant_kg")[-1]),
        }
    ax.set_ylim(-0.5, 12)
    ax.legend(frameon=False, loc="center right", bbox_to_anchor=(1.0, 0.42), labelcolor=INK)
    fig.tight_layout()
    fig.savefig(OUT / "altitude_comparison.png")
    plt.close(fig)
    return results


def _finite(o):
    """Replace inf/NaN (e.g. hover power of a vehicle that cannot hover) with null for strict JSON."""
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    if isinstance(o, float) and not np.isfinite(o):
        return None
    return o


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    mars = load_body("mars")
    msh_body = Body.with_surface_density(mars, density=0.015, temperature=223.15)
    report = {
        "conditions": {"mars_design_density_kg_m3": 0.015, "mars_design_temperature_c": -50.0},
        "hover": {
            "mars_hexacopter_tcc": hover_report(load_vehicle("mars_hexacopter_tcc"), msh_body).as_dict(),
            "tcc_mars_70kg_original": hover_report(load_vehicle("tcc_mars_70kg_original"), msh_body).as_dict(),
            "lunar_hopper_tcc": hover_report(load_vehicle("lunar_hopper_tcc"), load_body("moon")).as_dict(),
        },
        "lunar_hopper_delta_v_m_s": lunar_delta_v(230.0, 80.0, 45.0),
        "nasa_msh_reference_hover_power_w": 6200.0,
    }
    report["feasibility"] = feasibility_chart(msh_body)
    report["flights"] = flights()
    with open(OUT / "results.json", "w") as f:
        json.dump(_finite(report), f, indent=2, ensure_ascii=False)

    h = report["hover"]
    msh = h["mars_hexacopter_tcc"]
    print(f"MSH-class hexacopter hover power: {msh['hover_power_w']:.0f} W (NASA: 6200 W), "
          f"endurance {msh['endurance_min']:.1f} min, T/W {msh['thrust_to_weight']:.2f}")
    print(f"TCC 70 kg drone: T/W {h['tcc_mars_70kg_original']['thrust_to_weight']:.2f}, "
          f"max hover mass {report['feasibility']['tcc_70kg_max_hover_mass_kg']:.1f} kg")
    print(f"Lunar hopper: T/W {h['lunar_hopper_tcc']['thrust_to_weight']:.2f}, "
          f"hover endurance {h['lunar_hopper_tcc']['endurance_min']:.1f} min, "
          f"delta-v {report['lunar_hopper_delta_v_m_s']:.0f} m/s")
    for k, r in report["flights"].items():
        print(k, r)


if __name__ == "__main__":
    main()
