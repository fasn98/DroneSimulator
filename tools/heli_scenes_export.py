"""
Passo 4: telemetry of the HUD scenes of the Helicóptero UTI (Template A, classe H135), computed by the Twin.

The 3D scene (web/heli/) only replays these files: every position, attitude, NR, torque, power margin, SADPF
event and branch prediction shown was computed here by the physics model, not animated by hand.

    python -m tools.heli_scenes_export        (~2 min; run on an idle machine: the branch-prediction time is
                                               measured on the wall clock)
Writes web/heli/data/{cat_a,advisory,autorotation,mission}.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.mission import catA_fuel_and_radius, loading_for  # noqa: E402
from src.physics.helicopter.model import engine_limit_w  # noqa: E402
from src.physics.helicopter.procedures import evaluate_cat_a  # noqa: E402
from src.physics.helicopter.scenarios import (CatAConfig, autorotation, cat_a_run,  # noqa: E402
                                              rescue_site_config)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "web", "heli", "data")
RES = os.path.join(ROOT, "docs", "helicoptero")
DISCLAIMER = "Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante."
RATING_NAMES = ["Decolagem (AEO)", "OEI 30 s", "OEI 2 min", "OEI contínuo"]
RATING_CODES = ["TO", "OEI30", "OEI2", "OEIC"]

# telemetry columns exported per frame (0.1 s) and their rounding
COLS = {"t": 2, "x": 2, "y": 2, "z": 2, "vx": 2, "vy": 2, "vz": 2, "roll_deg": 2, "pitch_deg": 2, "yaw_deg": 2,
        "nr_pct": 1, "ias_kt": 1, "tas_ms": 2, "p_eng1_kw": 1, "p_eng2_kw": 1, "p_avail_kw": 1, "p_main_kw": 1, "p_tail_kw": 1,
        "mass_kg": 1, "fuel_kg": 2, "on_ground": 0, "rating": 0, "vrs": 0, "agl": 2, "sadpf_level": 0,
        "collective_deg": 2}


def _r(v, nd):
    v = float(v)
    if not math.isfinite(v):
        return None
    return round(v, nd) if nd else int(round(v))


def _clean(o):
    """JSON-safe copy (numpy scalars, NaN -> None)."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items() if k not in ("telemetry", "sim")}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not math.isfinite(float(o)) else round(float(o), 4)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    return o


def frames(tel, sim) -> Dict[str, list]:
    """Column-oriented frames + derived HUD quantities (all from the simulation)."""
    cols = {k: [_r(v, nd) for v in tel.column(k)] for k, nd in COLS.items()}
    p = sim.p
    nr = np.maximum(tel.column("nr_pct"), 1.0) / 100.0
    # engine torque in % of the 100 % torque (665 N m at the gearbox input): P / (P100 * NR) (DERIVADO)
    for i in (1, 2):
        cols[f"tq{i}_pct"] = [_r(v, 1) for v in tel.column(f"p_eng{i}_kw") / (p.p100_kw * nr) * 100.0]
    req = tel.column("p_eng1_kw") + tel.column("p_eng2_kw")
    n_run = np.where(tel.column("rating") > 0, 1, 2)
    cols["margin_kw"] = [_r(v, 1) for v in tel.column("p_avail_kw") - req]
    # fuel flow and endurance with the fuel on board at the current power (Willans line, CALIBRADO)
    from src.physics.helicopter.model import fuel_flow_kgs
    ff = np.array([fuel_flow_kgs(p, max(pw, 0.0) * 1e3, int(n)) for pw, n in zip(req, n_run)])
    cols["ff_kgh"] = [_r(v * 3600, 1) for v in ff]
    cols["endurance_min"] = [_r(f / max(q, 1e-6) / 60.0, 0) for f, q in zip(tel.column("fuel_kg"), ff)]
    cols["skid_h"] = [_r(v - p.cg_h, 2) for v in tel.column("z")]  # skid height above the deck / ground level
    cols["agl_skid"] = [_r(v - p.cg_h, 2) for v in tel.column("agl")]
    return cols


def torque_limits(sim) -> Dict[str, float]:
    """Per-engine torque limit (% of 100 % torque) of each rating at the scene's density altitude (DERIVADO from
    the TCDS torque limits and the engine ratings, see docs/helicoptero-uti.md)."""
    out = {}
    for code, n in (("TO", 2), ("OEI30", 1), ("OEI2", 1), ("OEIC", 1)):
        out[code] = round(engine_limit_w(sim.p, sim.atm, sim.x[2], code, n) / 1e3 / sim.p.p100_kw * 100.0, 1)
    return out


def events_of(sim) -> List[dict]:
    ev = []
    for e in sim.events:
        d = {"t": round(float(e["t"]), 2), "code": e.get("code"), "level": e.get("level", 1),
             "message": e.get("message", "")}
        if e.get("advisory"):
            d["advisory"] = True
            d["alert"] = bool(e.get("alert"))
        ev.append(d)
    return ev


def _advisory_view(a: Optional[dict]) -> Optional[dict]:
    if not a:
        return None
    return _clean({k: a[k] for k in ("label", "t", "procedure_action", "predicted", "advise", "alert", "mode",
                                      "wall_time_s", "wall_time_each_s", "parallel_estimate_s",
                                      "sequential_estimate_s", "within_budget")})


def _run_view(r: dict, cfg: CatAConfig, name: str, extra: Optional[dict] = None) -> dict:
    sim, tel = r["sim"], r["telemetry"]
    ev = {k: v for k, v in r.items() if k not in ("telemetry", "sim", "advisory", "phases")}
    out = {"name": name, "frames": frames(tel, sim), "events": events_of(sim), "phases": _clean(r["phases"]),
           "evaluation": _clean(ev), "advisory": _advisory_view(r.get("advisory")),
           "fail_time": sim.fail_time, "torque_limits_pct": torque_limits(sim),
           "omega100": sim.p.omega100, "rotor_radius_m": sim.p.R, "cg_h": sim.p.cg_h}
    out.update(extra or {})
    return out


def _cg_for(mass: float, patient: bool = True) -> dict:
    l = loading_for(patient)
    fuel = max(mass - l.zero_fuel_mass, 0.0)
    return {"fuel_kg": round(fuel, 1), **{k: (v if isinstance(v, bool) else round(v, 1)) for k, v in l.cg(fuel).items()}}


def scene_cat_a() -> dict:
    """(a) Categoria A, heliponto elevado (30 m), 1.500 m ISA+25, na massa máxima Cat A (G = 1,26)."""
    b = json.load(open(os.path.join(RES, "passo2b_resultados.json"), encoding="utf-8"))
    p3 = json.load(open(os.path.join(RES, "passo3_resultados.json"), encoding="utf-8"))
    cond = (1500.0, 25.0, 0.0)
    m29_60 = next(r["mass_kg"] for r in b["cat_a_max_mass"] if r["criterion"] == "elevado_29_60"
                  and (r["elevation_m"], r["delta_t"], r["headwind_ms"]) == cond)
    m29_59 = next(r["mass_kg"] for r in b["cat_a_max_mass"] if r["criterion"] == "literal_29_59c"
                  and (r["elevation_m"], r["delta_t"], r["headwind_ms"]) == cond)
    row = next(c for c in p3["conditions"] if (c["elevation_m"], c["delta_t"], c["headwind_ms"]) == cond)
    mass = m29_60
    cg = _cg_for(mass)
    cfg = CatAConfig(elevation_m=cond[0], delta_t=cond[1], params={"fuel": cg["fuel_kg"]})
    runs = {}
    for branch in ("reject", "continue"):
        r = cat_a_run(mass, branch, cfg=cfg, advisory=True)
        # the same flight judged with the literal 14 CFR 29.59(c) criterion, shown side by side in the HUD
        lit = evaluate_cat_a(r["telemetry"], r["sim"], _vtoss_of(r), branch, "literal_29_59c")
        runs[branch] = _run_view(r, cfg, branch, {"literal_29_59c": _clean(lit)})
    hv = b["hv"]["1.500 m, ISA+25, 2614 kg"]
    return {"scene": "cat_a", "disclaimer": DISCLAIMER,
            "title": "Categoria A · heliponto elevado · falha de motor",
            "subtitle": "Helicóptero UTI · bimotor classe H135 (genérico)",
            "condition": {"label": "1.500 m, ISA+25 (40 °C), sem vento", "elevation_m": cond[0], "delta_t": cond[1],
                          "deck_height_m": cfg.deck_height_m, "deck_size_m": 2 * cfg.deck_half_size_m,
                          "tdp_height_m": cfg.tdp_height_m},
            "mass_kg": mass,
            "cat_a_mass": {"g126_kg": row["cat_a_mass_kg"], "g115_kg": row["cat_a_mass_g115_kg"],
                           "literal_29_59c_kg": m29_59,
                           "note": "faixa pelo ganho do duto do rotor de cauda (Fenestron) G = 1,15–1,26"},
            "cg": cg, "runs": runs,
            "hv": {"condition": "1.500 m, ISA+25, 2.614 kg (decolagem do solo)",
                   "points": [{"h": p["height_m"], "v": p["speed_kt"], "safe": p["safe"]} for p in hv["points"]]},
            "rating_names": RATING_NAMES, "rating_codes": RATING_CODES}


def _vtoss_of(r: dict) -> Optional[float]:
    ev = r.get("vtoss_kt")
    if ev:
        return ev
    from src.physics.helicopter.procedures import vtoss
    sim = r["sim"]
    return vtoss(sim.p, sim.atm, 0.0, r["mass_kg"])


def scene_advisory() -> dict:
    """(b) Previsão de ramos: local de resgate, 1.500 m ISA+25, 2.600,7 kg, falha 0,5 m acima do TDP.
    Procedure as the default (alert only displayed) and, for comparison, the pilot following the alert."""
    lr = json.load(open(os.path.join(RES, "local_resgate.json"), encoding="utf-8"))
    p3c = json.load(open(os.path.join(RES, "passo3c_resultados.json"), encoding="utf-8"))
    case = next(c for c in lr if c["condition"] == "1.500 m, ISA+25" and c["mass_kg"] > 2500)
    mass = case["mass_kg"]
    site_limit = next(r["mass_kg"] for r in p3c["rescue_max_mass"]
                      if r["condition"] == "1.500 m, ISA+25" and r["tdp_height_m"] == 12.0)
    cg = _cg_for(mass)
    base = rescue_site_config(1500.0, 25.0, 0.0)
    cfg = CatAConfig(**{**base.__dict__, "params": {"fuel": cg["fuel_kg"]}})
    runs = {}
    for name, follow in (("procedimento", False), ("consultivo_seguido", True)):
        r = cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=0.5, advisory=True, follow_advisory=follow)
        runs[name] = _run_view(r, cfg, name)
    sweep = {k: v for k, v in p3c.get("advisory_sweep", {}).items() if k != "rows"}
    return {"scene": "advisory", "disclaimer": DISCLAIMER,
            "title": "Previsão de ramos · alerta consultivo",
            "subtitle": "Helicóptero UTI · decolagem do local de resgate com o paciente",
            "condition": {"label": "local de resgate plano, 1.500 m, ISA+25, sem vento", "elevation_m": 1500.0,
                          "delta_t": 25.0, "deck_height_m": 0.0, "tdp_height_m": cfg.tdp_height_m},
            "mass_kg": mass, "site_limit_kg": site_limit, "cg": cg,
            "note": (f"Caso de demonstração: {mass:.0f} kg está acima da massa máxima do local "
                     f"({site_limit:.0f} kg, TDP 12 m). Com a limitação padrão este caso não ocorre."),
            "sweep_summary": _clean(sweep), "runs": runs,
            "rating_names": RATING_NAMES, "rating_codes": RATING_CODES}


def scene_autorotation() -> dict:
    """(c) Autorrotação a partir de 300 m: vertical × com velocidade à frente (mínima razão de descida)."""
    out = {}
    for mode in ("vertical", "forward"):
        r = autorotation(mode, height=300.0)
        sim, tel = r["sim"], r["telemetry"]
        out[mode] = {"name": mode, "frames": frames(tel, sim), "events": events_of(sim),
                     "phases": _clean(r["phases"]), "landing": _clean(r["landing"]),
                     "rod_steady_ms": r["rod_steady_ms"], "rod_predicted_ms": r["rod_predicted_ms"],
                     "v_glide_kt": r["v_glide_kt"], "min_nr_pct": r["min_nr_pct"], "sadpf": _clean(r["sadpf"]),
                     "torque_limits_pct": torque_limits(sim), "omega100": sim.p.omega100,
                     "rotor_radius_m": sim.p.R, "cg_h": sim.p.cg_h}
    return {"scene": "autorotation", "disclaimer": DISCLAIMER,
            "title": "Autorrotação · falha dos dois motores a 300 m",
            "subtitle": "Helicóptero UTI · vertical × com velocidade à frente",
            "condition": {"label": "nível do mar, ISA, sem vento", "elevation_m": 0.0, "delta_t": 0.0},
            "goal": {"sink_ms": 1.5, "ground_speed_kt": 15.0,
                     "note": "metas de toque ESTIMADAS (docs/helicoptero-uti.md)"},
            "flare_limitation": ("Limitação conhecida: o flare usa uma lei simples de cíclico e coletivo; a varredura "
                                 "do Passo 2 não atingiu toque ≤ 1,5 m/s com ≤ 15 kt (melhor: 1,44 m/s e 27,5 kt). "
                                 "Flare coordenado está no backlog."),
            "runs": out, "rating_names": RATING_NAMES, "rating_codes": RATING_CODES}


def mission_panel() -> dict:
    """(d) Mission panel data: every condition and profile (from docs/helicoptero/passo3*_resultados.json)."""
    p3 = json.load(open(os.path.join(RES, "passo3_resultados.json"), encoding="utf-8"))
    rows = []
    for c in p3["conditions"]:
        prof = {}
        for k, v in c["profiles"].items():
            prof[k] = {"fuel_kg": v["fuel_kg"], "fuel_limited_by": v["fuel_limited_by"], "radius_nm": v["radius_nm"],
                       "radius_nm_g115": v.get("radius_nm_g115"), "takeoff_mass_kg": v["takeoff_mass_kg"],
                       "return_takeoff_mass_kg": v["return_takeoff_mass_kg"],
                       "return_takeoff_mass_max_kg": v["return_takeoff_mass_max_kg"],
                       "flight_time_min": v["flight_time_min"], "reserve_kg": v["reserve_kg"],
                       "site_limit_kg": v.get("site_limit_kg"), "site_fuel_max_kg": v.get("site_fuel_max_kg"),
                       "site_min_radius_full_fuel_nm": v.get("site_min_radius_full_fuel_nm"),
                       "tdp17": v.get("tdp17"), "aux_tank": v.get("aux_tank")}
        rows.append({"condition": c["condition"], "cat_a_mass_kg": c["cat_a_mass_kg"],
                     "cat_a_mass_g115_kg": c["cat_a_mass_g115_kg"], "site_limit_tdp12_kg": c["site_limit_tdp12_kg"],
                     "site_limit_tdp17_kg": c["site_limit_tdp17_kg"], "profiles": prof})
    return _clean({"profiles": {"resgate": "Resgate (ida sem paciente, volta com paciente)",
                                "transferencia": "Transferência (ida com paciente, volta sem)",
                                "conservador": "Conservador (paciente nos dois trechos)"},
                   "default_profile": "resgate", "reserve_min": 20,
                   "reserve_source": "RBAC 91.151(b) (padrão provisório)",
                   "cruise_reference": "cruzeiro em ISA, 1.000 ft (condição de referência comum)",
                   "conditions": rows})


def main():
    os.makedirs(OUT, exist_ok=True)
    which = sys.argv[1:] or ["mission", "cat_a", "advisory", "autorotation"]
    makers = {"mission": mission_panel, "cat_a": scene_cat_a, "advisory": scene_advisory,
              "autorotation": scene_autorotation}
    for name in which:
        data = makers[name]()
        with open(os.path.join(OUT, f"{name}.json"), "w", encoding="utf-8") as f:
            json.dump(_clean(data), f, ensure_ascii=False, separators=(",", ":"))
        print(name, "ok", os.path.getsize(os.path.join(OUT, f"{name}.json")) // 1024, "kB")


if __name__ == "__main__":
    main()
