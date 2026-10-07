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
from src.physics.helicopter.params import v as param  # noqa: E402
from src.physics.helicopter.procedures import (TD_LIMIT_MS, TD_RESERVE_MS, TD_SEAT_MS,  # noqa: E402
                                               evaluate_cat_a, evaluate_landing)
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
        "collective_deg": 2, "n_eng": 0}


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


NR_LIMITS = {"power_on": [param("nr_power_on_min"), param("nr_power_on_max")],
             "power_off": [param("nr_power_off_min"), param("nr_power_off_max")],
             "source": "TCDS EASA R.009 (limites de NR com e sem motor; ver docs/fontes.md)"}
TOUCHDOWN = {"limit_29_725_ms": TD_LIMIT_MS, "reserve_29_727_ms": TD_RESERVE_MS, "seat_29_562_ms": TD_SEAT_MS}


def nr_exceedances(tel) -> List[dict]:
    """Episodes outside the NR limits: power-on limits while an engine runs, power-off limits after both failed.
    The lower limit is checked only in flight (on the ground the rotor is expected to slow down)."""
    t, nr = tel.column("t"), tel.column("nr_pct")
    n_eng, ground = tel.column("n_eng"), tel.column("on_ground") > 0
    eps, cur = [], None
    for i in range(len(t)):
        lo, hi = NR_LIMITS["power_on"] if n_eng[i] >= 1 else NR_LIMITS["power_off"]
        regime = "com motor" if n_eng[i] >= 1 else "sem motor"
        kind = "acima" if nr[i] > hi else ("abaixo" if (nr[i] < lo and not ground[i]) else None)
        if kind and cur and cur["kind"] == kind and cur["regime"] == regime:
            cur["t1"] = float(t[i])
            cur["peak"] = float(max(cur["peak"], nr[i]) if kind == "acima" else min(cur["peak"], nr[i]))
        elif kind:
            cur = {"kind": kind, "regime": regime, "limit": float(hi if kind == "acima" else lo), "t0": float(t[i]),
                   "t1": float(t[i]), "peak": float(nr[i]), "on_ground": bool(ground[i])}
            eps.append(cur)
        else:
            cur = None
    for e in eps:
        e["duration_s"] = round(e["t1"] - e["t0"] + 0.1, 2)  # 0,1 s per telemetry record
    return eps


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
           "landing": _clean(evaluate_landing(tel, t_from=sim.fail_time, min_air_s=1.0)),
           "nr_exceedances": nr_exceedances(tel),
           "evaluation": _clean(ev), "advisory": _advisory_view(r.get("advisory")),
           "fail_time": sim.fail_time, "torque_limits_pct": torque_limits(sim),
           "omega100": sim.p.omega100, "rotor_radius_m": sim.p.R, "cg_h": sim.p.cg_h}
    out.update(extra or {})
    return out


def _cg_for(mass: float, patient: bool = True) -> dict:
    l = loading_for(patient)
    fuel = max(mass - l.zero_fuel_mass, 0.0)
    return {"fuel_kg": round(fuel, 1), **{k: (v if isinstance(v, bool) else round(v, 1)) for k, v in l.cg(fuel).items()}}


def _recalc(v: str) -> dict:
    """Recomputed Category A results (tools/heli_reject_recalc.py) for reject procedure v1 or v2."""
    with open(os.path.join(RES, f"recalculo_abortar_{v}.json"), encoding="utf-8") as f:
        return json.load(f)


def _catA_mass(rc: dict, cond, criterion="elevado_29_60", g=1.26):
    return next(r["mass_kg"] for r in rc["catA"] if r["criterion"] == criterion and r["duct_gain"] == g
                and (r["elevation_m"], r["delta_t"], r["headwind_ms"]) == tuple(cond))


def scene_cat_a() -> dict:
    """(a) Categoria A, heliponto elevado (30 m), 1.500 m ISA+25, na massa máxima Cat A (G = 1,26) do procedimento
    de abortar padrão (v2, com amortecimento); o v1 (sem amortecimento) aparece como referência conservadora."""
    b = json.load(open(os.path.join(RES, "passo2b_resultados.json"), encoding="utf-8"))
    cond = (1500.0, 25.0, 0.0)
    v1, v2 = _recalc("v1"), _recalc("v2")
    masses = {v: {"g126_kg": _catA_mass(rc, cond), "g115_kg": _catA_mass(rc, cond, g=1.15),
                  "literal_29_59c_kg": _catA_mass(rc, cond, "literal_29_59c")} for v, rc in (("v1", v1), ("v2", v2))}
    old = {"g126_kg": next(r["mass_kg"] for r in b["cat_a_max_mass"] if r["criterion"] == "elevado_29_60"
                           and (r["elevation_m"], r["delta_t"], r["headwind_ms"]) == cond)}
    mass = masses["v2"]["g126_kg"]
    cg = _cg_for(mass)
    runs = {}
    for name, branch, proc in (("reject", "reject", "v2"), ("continue", "continue", "v2"), ("reject_v1", "reject", "v1")):
        cfg = CatAConfig(elevation_m=cond[0], delta_t=cond[1], params={"fuel": cg["fuel_kg"]}, reject_procedure=proc)
        r = cat_a_run(mass, branch, cfg=cfg, advisory=True)
        # the same flight judged with the literal 14 CFR 29.59(c) criterion, shown side by side in the HUD
        lit = evaluate_cat_a(r["telemetry"], r["sim"], _vtoss_of(r), branch, "literal_29_59c")
        runs[name] = _run_view(r, cfg, name, {"literal_29_59c": _clean(lit), "reject_procedure": proc})
    hv = b["hv"]["1.500 m, ISA+25, 2614 kg"]
    return {"scene": "cat_a", "disclaimer": DISCLAIMER,
            "title": "Categoria A · heliponto elevado · falha de motor",
            "subtitle": "Helicóptero UTI · bimotor classe H135 (genérico)",
            "condition": {"label": "1.500 m, ISA+25 (40 °C), sem vento", "elevation_m": cond[0], "delta_t": cond[1],
                          "deck_height_m": 30.0, "deck_size_m": 20.0, "tdp_height_m": 12.0},
            "mass_kg": mass,
            "cat_a_mass": {**masses["v2"], "v1": masses["v1"], "old_invalid_g126_kg": old["g126_kg"],
                           "note": "faixa pelo ganho do duto do rotor de cauda (Fenestron) G = 1,15–1,26"},
            "reject_procedures": {"v2": "procedimento de abortar v2 (com amortecimento), padrão",
                                  "v1": "procedimento de abortar v1 (sem amortecimento), referência conservadora"},
            "cg": cg, "runs": runs,
            "hv": {"condition": "1.500 m, ISA+25, 2.614 kg (decolagem do solo)",
                   "points": [{"h": p["height_m"], "v": p["speed_kt"], "safe": p["safe"]} for p in hv["points"]]},
            "rating_names": RATING_NAMES, "rating_codes": RATING_CODES, "nr_limits": NR_LIMITS,
            "touchdown_thresholds": TOUCHDOWN}


def _vtoss_of(r: dict) -> Optional[float]:
    ev = r.get("vtoss_kt")
    if ev:
        return ev
    from src.physics.helicopter.procedures import vtoss
    sim = r["sim"]
    return vtoss(sim.p, sim.atm, 0.0, r["mass_kg"])


def scene_advisory() -> dict:
    """(b) Previsão de ramos, com o abortar v2 (padrão). The case shown is the first alert of the 130-case check of
    the v2 recalculation, preferring the rescue site (the case of the Passo 3, 1.500 m ISA+25, 2.600,7 kg, falha
    0,5 m acima do TDP, when it still alerts). Procedure as the default (alert only displayed) and, for comparison,
    the pilot following the alert."""
    rc = _recalc("v2")
    rows = rc["advisory"]["rows"]
    alerts = [r for r in rows if r["alert"]]
    pick = next((r for r in alerts if r["kind"] == "local de resgate" and r["condition"] == "1.500 m, ISA+25"
                 and abs(r["fail_rel_tdp_m"] - 0.5) < 1e-6), None) \
        or next((r for r in alerts if r["kind"] == "local de resgate"), None) or (alerts[0] if alerts else None)
    if pick is None:  # no alert at all with v2: show the Passo 3 case, without alert
        pick = next(r for r in rows if r["kind"] == "local de resgate" and r["condition"] == "1.500 m, ISA+25"
                    and abs(r["fail_rel_tdp_m"] - 0.5) < 1e-6)
    mass, rel = pick["mass_kg"], pick["fail_rel_tdp_m"]
    elevated = pick["kind"] == "heliponto elevado"
    if elevated:
        cond = (0.0, 0.0, 0.0) if pick["condition"].startswith("nivel") else (1500.0, 25.0, 0.0)
        base = CatAConfig(elevation_m=cond[0], delta_t=cond[1])
        label = f"heliponto elevado, {'nível do mar, ISA' if cond[0] == 0 else '1.500 m, ISA+25'}, sem vento"
        site_limit = None
    else:
        c = next(c for c, n in zip([(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0),
                                    (1500.0, 25.0, 8.0)],
                                   ["nível do mar, ISA", "nível do mar, ISA+20", "1.000 m, ISA+20", "1.500 m, ISA+25",
                                    "1.500 m, ISA+25, proa 8 m/s"]) if n == pick["condition"])
        base = rescue_site_config(*c)
        label = f"local de resgate plano, {pick['condition']}"
        site_limit = next(r["mass_kg"] for r in rc["site"] if r["condition"] == pick["condition"]
                          and r["tdp_height_m"] == 12.0)
    cg = _cg_for(mass)
    cfg = CatAConfig(**{**base.__dict__, "params": {"fuel": cg["fuel_kg"]}, "reject_procedure": "v2"})
    runs = {}
    for name, follow in (("procedimento", False), ("consultivo_seguido", True)):
        r = cat_a_run(mass, cfg=cfg, fail_rel_tdp_m=rel, advisory=True, follow_advisory=follow)
        runs[name] = _run_view(r, cfg, name)
    sweep = {k: v for k, v in rc["advisory"].items() if k != "rows"}
    note = (f"Caso de demonstração: {mass:.0f} kg, acima da massa máxima do local ({site_limit:.0f} kg, TDP 12 m, "
            f"abortar v2)." if site_limit and mass > site_limit else "")
    return {"scene": "advisory", "disclaimer": DISCLAIMER,
            "title": "Previsão de ramos · alerta consultivo",
            "subtitle": "Helicóptero UTI · " + ("decolagem do heliponto elevado" if elevated else
                                                "decolagem do local de resgate com o paciente"),
            "condition": {"label": label, "elevation_m": base.elevation_m, "delta_t": base.delta_t,
                          "deck_height_m": base.deck_height_m, "tdp_height_m": cfg.tdp_height_m},
            "mass_kg": mass, "site_limit_kg": site_limit, "cg": cg, "fail_rel_tdp_m": rel, "note": note,
            "has_alert": bool(pick["alert"]), "sweep_summary": _clean(sweep), "runs": runs,
            "rating_names": RATING_NAMES, "rating_codes": RATING_CODES, "nr_limits": NR_LIMITS,
            "touchdown_thresholds": TOUCHDOWN}


def scene_autorotation() -> dict:
    """(c) Autorrotação a partir de 300 m: vertical × com velocidade à frente (mínima razão de descida)."""
    out = {}
    for mode in ("vertical", "forward"):
        r = autorotation(mode, height=300.0)
        sim, tel = r["sim"], r["telemetry"]
        out[mode] = {"name": mode, "frames": frames(tel, sim), "events": events_of(sim),
                     "phases": _clean(r["phases"]), "landing": _clean(r["landing"]),
                     "nr_exceedances": nr_exceedances(tel),
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
            "runs": out, "rating_names": RATING_NAMES, "rating_codes": RATING_CODES, "nr_limits": NR_LIMITS,
            "touchdown_thresholds": TOUCHDOWN}


def mission_panel() -> dict:
    """(d) Mission panel data: every condition and profile, with the Category A and rescue-site masses of the
    reject procedure v2 (default) and, for reference, v1 (docs/helicoptero/recalculo_abortar_v*.json)."""
    v1 = {r["condition"]: r for r in _recalc("v1")["mission"]}
    rows = []
    for c in _recalc("v2")["mission"]:
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
        r1 = v1[c["condition"]]
        rows.append({"condition": c["condition"], "cat_a_mass_kg": c["cat_a_mass_kg"],
                     "cat_a_mass_g115_kg": c["cat_a_mass_g115_kg"], "site_limit_tdp12_kg": c["site_limit_tdp12_kg"],
                     "site_limit_tdp17_kg": c["site_limit_tdp17_kg"], "profiles": prof,
                     "v1": {"cat_a_mass_kg": r1["cat_a_mass_kg"], "cat_a_mass_g115_kg": r1["cat_a_mass_g115_kg"],
                            "site_limit_tdp12_kg": r1["site_limit_tdp12_kg"],
                            "radius_nm": {k: v["radius_nm"] for k, v in r1["profiles"].items()}}})
    return _clean({"profiles": {"resgate": "Resgate (ida sem paciente, volta com paciente)",
                                "transferencia": "Transferência (ida com paciente, volta sem)",
                                "conservador": "Conservador (paciente nos dois trechos)"},
                   "default_profile": "resgate", "reserve_min": 20,
                   "reject_procedure": "massas com o procedimento de abortar v2 (com amortecimento); v1 como referência",
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
