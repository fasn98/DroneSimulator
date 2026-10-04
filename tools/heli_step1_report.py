"""Passo 1 report: key numbers and the power-curve figure of the Helicóptero UTI template.

python tools/heli_step1_report.py  ->  docs/helicoptero/passo1_resultados.json, docs/helicoptero/curva_potencia.png
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.physics.helicopter import (EngineFault, HeliAtmosphere, HeliParams, HelicopterSimulator, best_speeds,  # noqa: E402
                                    calibrate_drag_area, level_flight, power_curve, scripted)
from src.physics.helicopter.isa import isa_density  # noqa: E402
from src.physics.helicopter.model import engine_limit_w  # noqa: E402

OUT = ROOT / "docs" / "helicoptero"


def hold(z):
    return lambda t, x: (np.array([0.0, 0.0, z]), np.zeros(3), 0.0)


def main():
    p = HeliParams()
    p.f_drag = calibrate_drag_area(p)
    h = level_flight(p, 0.0, 1.225)
    ige = level_flight(p, 0.0, 1.225, z_hub=p.cg_h + p.hub_h)
    fm = h.thrust ** 1.5 / math.sqrt(2 * 1.225 * p.area) / h.p_main
    rho72 = isa_density(7200 * 0.3048)
    req72 = level_flight(p, 0.0, rho72).p_engines
    av72 = 2 * engine_limit_w(p, HeliAtmosphere(), 7200 * 0.3048, "TO", 2)
    bs = best_speeds(p, 1.225)
    # NR decay with no pilot action and steady autorotation
    sim = HelicopterSimulator(wind=False, start_position=(0, 0, 500.0), engine_faults=[EngineFault(0, 1.0), EngineFault(1, 1.0)])
    sim.collective_hold = sim.u.collective
    tel = sim.run(6.0, hold(500.0), 0.02)
    t, nr = tel.column("t"), tel.column("nr_pct")
    t85 = float(t[np.argmax(nr < 85.0)] - 1.0)
    sim = HelicopterSimulator(wind=False, start_position=(0, 0, 800.0), engine_faults=[EngineFault(0, 1.0), EngineFault(1, 1.0)])
    tel = sim.run(25.0, scripted(hold(800.0), 2.0, lambda: sim.set_autorotation(True)), 0.1)
    st = tel.column("t") > 12
    res = {
        "f_drag_m2 (CALIBRADO)": p.f_drag,
        "hover_oge_sl_mtow_kw": h.p_engines / 1e3, "hover_main_rotor_kw": h.p_main / 1e3, "hover_tail_rotor_kw": h.p_tail / 1e3,
        "tail_over_main": h.p_tail / h.p_main, "figure_of_merit": fm, "hover_ige_on_skids_kw": ige.p_engines / 1e3,
        "ige_reduction": 1 - ige.p_engines / h.p_engines, "aeo_takeoff_available_sl_kw": p.gearbox_limit_kw("TO"),
        "hoge_7200ft_required_kw": req72 / 1e3, "hoge_7200ft_available_kw": av72 / 1e3,
        "hoge_7200ft_divergence": req72 / av72 - 1, **bs,
        "nr_power_off_t_to_85pct_s": t85, "autorotation_vz_ms": float(tel.column("vz")[st].mean()),
        "autorotation_nr_min": float(tel.column("nr_pct")[st].min()), "autorotation_nr_max": float(tel.column("nr_pct")[st].max()),
        "oei30_available_kw": p.gearbox_limit_kw("OEI30"), "oei2_available_kw": p.gearbox_limit_kw("OEI2"),
        "oeic_available_kw": p.gearbox_limit_kw("OEIC"),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "passo1_resultados.json").write_text(json.dumps(res, indent=2, default=float))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pc = power_curve(p, 1.225, v_max_kt=145, n=59)
    v = np.array([q.V for q in pc]) / 0.514444
    fig, ax = plt.subplots(figsize=(8, 4.6), dpi=150)
    ax.plot(v, [q.p_engines / 1e3 for q in pc], color="#1d4f8f", lw=2.2, label="Total requerida (motores)")
    ax.plot(v, [q.p_induced / 1e3 for q in pc], color="#6b8fc7", lw=1.2, ls="--", label="Induzida (κ·T·vᵢ)")
    ax.plot(v, [q.p_profile / 1e3 for q in pc], color="#8a6d3b", lw=1.2, ls="--", label="Perfil (1 + 4,65 μ²)")
    ax.plot(v, [q.p_parasite / 1e3 for q in pc], color="#a3361f", lw=1.2, ls="--", label="Parasita (½ρV³f)")
    ax.plot(v, [q.p_tail / 1e3 for q in pc], color="#0f6b5c", lw=1.2, ls=":", label="Rotor de cauda")
    for key, lab, c in (("TO", "AEO decolagem", "#333333"), ("MCP", "AEO máx. contínua", "#777777"),
                        ("OEI30", "OEI 30 s", "#b35c00"), ("OEIC", "OEI contínua", "#d4a017")):
        ax.axhline(p.gearbox_limit_kw(key), color=c, lw=0.9, ls="-.", label=f"{lab}: {p.gearbox_limit_kw(key):.0f} kW")
    ax.axvline(bs["v_be_kt"], color="#1d4f8f", lw=0.7, alpha=0.5)
    ax.axvline(bs["v_br_kt"], color="#1d4f8f", lw=0.7, alpha=0.5)
    ax.text(bs["v_be_kt"], 40, f" V máx. autonomia\n {bs['v_be_kt']:.0f} kt", fontsize=7)
    ax.text(bs["v_br_kt"], 40, f" V máx. alcance\n {bs['v_br_kt']:.0f} kt", fontsize=7)
    ax.set_xlabel("Velocidade verdadeira (kt)")
    ax.set_ylabel("Potência (kW)")
    ax.set_title("Classe H135 · MTOW 2.980 kg · ISA nível do mar · calculado pelo Twin", fontsize=10)
    ax.set_ylim(0, 700)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=6.5, ncol=2, loc="upper center")
    fig.text(0.01, 0.01, "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.",
             fontsize=6, color="#555555")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(OUT / "curva_potencia.png")
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
