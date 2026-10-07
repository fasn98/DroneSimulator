"""
Comparison of the reject-procedure recalculation (aval do Passo 4): old results (invalidated: touchdown taken at
the lift-off skid bounce), v1 (sem amortecimento) and v2 (com amortecimento), side by side.

    python -m tools.heli_reject_report
Writes docs/helicoptero/recalculo_abortar_comparacao.json and the tables of docs/helicoptero-uti.md between the
RECALC markers.
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "docs", "helicoptero")
DOC = os.path.join(ROOT, "docs", "helicoptero-uti.md")
CONDS = [(0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (1000.0, 20.0, 0.0), (1500.0, 25.0, 0.0), (1500.0, 25.0, 8.0)]
NAMES = ["nível do mar, ISA", "nível do mar, ISA+20", "1.000 m, ISA+20", "1.500 m, ISA+25", "1.500 m, ISA+25, proa 8 m/s"]
PROFILE_NAMES = {"resgate": "Resgate", "transferencia": "Transferência", "conservador": "Conservador"}


def load(name):
    with open(os.path.join(RES, name), encoding="utf-8") as f:
        return json.load(f)


def kg(v):
    return "—" if v is None else f"{v:,.0f}".replace(",", ".")


def nm(v):
    return "—" if v is None else f"{v:,.0f}".replace(",", ".")


def diff(new, old):
    if new is None or old is None:
        return "—"
    d = new - old
    return f"{d:+,.0f}".replace(",", ".")


def key(r):
    return (r["elevation_m"], r["delta_t"], r["headwind_ms"])


def sweep_summary(rows, height_key, tdp=12.0):
    """Failure heights (relative to the TDP for the elevated heliport) where each branch is safe."""
    out = {"abortar_seguro": [], "prosseguir_seguro": [], "nenhum": []}
    for r in rows:
        h = r[height_key]
        if r["reject"]["safe"]:
            out["abortar_seguro"].append(h)
        if r["continue"]["safe"]:
            out["prosseguir_seguro"].append(h)
        if not (r["reject"]["safe"] or r["continue"]["safe"]):
            out["nenhum"].append(h)
    return out


def fmt_heights(hs):
    if not hs:
        return "nenhuma"
    return ", ".join(f"{h:g}".replace(".", ",") for h in hs)


def main():
    b2, sens, p3, p3c, lr = (load(n) for n in ("passo2b_resultados.json", "sensibilidade_antitorque.json",
                                               "passo3_resultados.json", "passo3c_resultados.json", "local_resgate.json"))
    v = {k: load(f"recalculo_abortar_{k}.json") for k in ("v1", "v2")
         if os.path.exists(os.path.join(RES, f"recalculo_abortar_{k}.json"))}
    old = {"catA": {(key(r), r["criterion"], 1.26): r["mass_kg"] for r in b2["cat_a_max_mass"]},
           "site": {(key(r), r["tdp_height_m"]): r["mass_kg"] for r in p3c["rescue_max_mass"]}}
    for r in sens["duct_gain_cat_a"]:
        if abs(r["duct_gain"] - 1.15) < 1e-9 and r.get("criterion", "elevado_29_60") == "elevado_29_60":
            old["catA"][(key(r), "elevado_29_60", 1.15)] = r["mass_kg"]
    new = {k: {"catA": {(key(r), r["criterion"], r["duct_gain"]): r["mass_kg"] for r in d.get("catA", [])},
               "catA_limit": {(key(r), r["criterion"], r["duct_gain"]): r["limited_by"] for r in d.get("catA", [])},
               "site": {(key(r), r["tdp_height_m"]): r["mass_kg"] for r in d.get("site", [])}} for k, d in v.items()}
    L = []
    vs = list(v)
    # 1. Category A masses
    L.append("**1. Massa máxima Categoria A** (heliponto elevado, kg). Faixa = G 1,15–1,26 (duto do Fenestron). "
             "Entre parênteses, a diferença do limite superior para o número antigo.\n")
    hdr = "| Condição | Antigo, 29.60 (inválido) | Antigo, literal 29.59(c) (inválido) |"
    sep = "|---|---|---|"
    for k in vs:
        hdr += f" {k}: 29.60 | {k}: literal 29.59(c) | {k}: limitado por |"
        sep += "---|---|---|"
    L += [hdr, sep]
    comp = {"catA": [], "site": [], "mission": [], "sweeps": {}, "advisory": {}}
    for c, n in zip(CONDS, NAMES):
        o126, o115, olit = (old["catA"].get((c, "elevado_29_60", 1.26)), old["catA"].get((c, "elevado_29_60", 1.15)),
                            old["catA"].get((c, "literal_29_59c", 1.26)))
        row = f"| {n} | {kg(o115)}–{kg(o126)} | {kg(olit)} |"
        crow = {"condition": n, "old": {"g115": o115, "g126": o126, "literal": olit}}
        for k in vs:
            m126, m115 = new[k]["catA"].get((c, "elevado_29_60", 1.26)), new[k]["catA"].get((c, "elevado_29_60", 1.15))
            mlit = new[k]["catA"].get((c, "literal_29_59c", 1.26))
            lim = new[k]["catA_limit"].get((c, "elevado_29_60", 1.26), "—")
            row += f" **{kg(m115)}–{kg(m126)}** ({diff(m126, o126)}) | {kg(mlit)} ({diff(mlit, olit)}) | {lim} |"
            crow[k] = {"g115": m115, "g126": m126, "literal": mlit, "limited_by": lim}
        L.append(row)
        comp["catA"].append(crow)
    # 2. rescue site
    L.append("\n**2. Massa máxima de decolagem do local de resgate** (kg; TDP 12 m, padrão, e 17 m, sensibilidade)\n")
    hdr = "| Condição | Antigo TDP 12 (17) (inválido) |" + "".join(f" {k}: TDP 12 (17) |" for k in vs)
    L += [hdr, "|---|---|" + "---|" * len(vs)]
    for c, n in zip(CONDS, NAMES):
        o12, o17 = old["site"].get((c, 12.0)), old["site"].get((c, 17.0))
        row = f"| {n} | {kg(o12)} ({kg(o17)}) |"
        crow = {"condition": n, "old": [o12, o17]}
        for k in vs:
            m12, m17 = new[k]["site"].get((c, 12.0)), new[k]["site"].get((c, 17.0))
            row += f" **{kg(m12)}** ({kg(m17)}) [{diff(m12, o12)}] |"
            crow[k] = [m12, m17]
        L.append(row)
        comp["site"].append(crow)
    # 3. mission
    L.append("\n**3. Raio de ação, tanque padrão (NM)**: combustível embarcável e o que o limita; entre parênteses, "
             "o raio com G = 1,15 quando difere. Resgate inclui o limite de massa no local (TDP 12 m).\n")
    hdr = "| Condição | Perfil | Antigo (inválido) |" + "".join(f" {k} |" for k in vs)
    L += [hdr, "|---|---|---|" + "---|" * len(vs)]
    oldm = {c["condition"]: c for c in p3["conditions"]}
    newm = {k: {r["condition"]: r for r in d.get("mission", [])} for k, d in v.items()}
    for n in NAMES:
        for prof in ("resgate", "transferencia", "conservador"):
            op = oldm[n]["profiles"][prof]
            cell = lambda p: (f"{nm(p['radius_nm'])}" + (f" ({nm(p['radius_nm_g115'])})" if p.get('radius_nm_g115') and
                              abs(p['radius_nm_g115'] - p['radius_nm']) > 0.5 else "") +
                              f" · {kg(p['fuel_kg'])} kg ({p['fuel_limited_by']})" +
                              (f" · local ≥ {nm(p['site_min_radius_full_fuel_nm'])} NM com tanque cheio"
                               if prof == "resgate" and (p.get('site_min_radius_full_fuel_nm') or 0) > 0.5 else ""))
            row = f"| {n} | {PROFILE_NAMES[prof]} | {cell(op)} |"
            crow = {"condition": n, "profile": prof, "old": {k2: op.get(k2) for k2 in ("radius_nm", "fuel_kg", "fuel_limited_by")}}
            for k in vs:
                if n in newm[k]:
                    pp = newm[k][n]["profiles"][prof]
                    row += f" **{cell(pp)}** |"
                    crow[k] = {k2: pp.get(k2) for k2 in ("radius_nm", "radius_nm_g115", "fuel_kg", "fuel_limited_by",
                                                          "site_min_radius_full_fuel_nm")}
                else:
                    row += " — |"
            L.append(row)
            comp["mission"].append(crow)
    # 4. sweeps
    L.append("\n**4. Varreduras de falha em torno do TDP.** Heliponto elevado: altura da falha em relação ao TDP (m) "
             "em que cada ramo é seguro, na massa antiga e na nova. Local de resgate: alturas de falha (m) sem "
             "nenhum ramo seguro (intervalo exposto).\n")
    hdr = "| Caso | Antigo (inválido) |" + "".join(f" {k} |" for k in vs)
    L += [hdr, "|---|---|" + "---|" * len(vs)]
    old_elev = {"nível do mar, ISA": b2["failure_height_sweep"]["nivel_do_mar_2980kg"],
                "1.500 m, ISA+25": b2["failure_height_sweep"]["1500m_isa25_2614kg"]}
    for cond_name, rows in old_elev.items():
        o = sweep_summary(rows, "fail_rel_tdp_m")
        for which in ("massa antiga", "massa nova"):
            row = f"| Heliponto elevado, {cond_name}, {which} |"
            row += (f" abortar seguro em {fmt_heights(o['abortar_seguro'])}; nenhum em {fmt_heights(o['nenhum'])} |"
                    if which == "massa antiga" else " — |")
            for k in vs:
                sw = next((s for s in v[k].get("sweep_elevated", []) if s["condition"] == cond_name and s["which"] == which), None)
                if sw is None:
                    row += " — |"
                    continue
                s = sweep_summary(sw["rows"], "fail_rel_tdp_m")
                row += f" {kg(sw['mass_kg'])} kg: abortar seguro em {fmt_heights(s['abortar_seguro'])}; nenhum em {fmt_heights(s['nenhum'])} |"
                comp["sweeps"].setdefault(f"elevado {cond_name} {which}", {})[k] = s
            comp["sweeps"].setdefault(f"elevado {cond_name} {which}", {})["old"] = o if which == "massa antiga" else None
            L.append(row)
    for n in NAMES:
        for which in ("raio máximo", "missão mais curta (mais pesada)"):
            oc = next((c for c in lr if c["condition"] == n and c["which"] == which), None)
            row = f"| Local de resgate, {n}, {which} |"
            row += (f" {kg(oc['mass_kg'])} kg: exposto em {fmt_heights([r['fail_skid_h_m'] for r in oc['sweep'] if r['exposed']])} |"
                    if oc else " — |")
            for k in vs:
                sc = next((c for c in v[k].get("sweep_site", []) if c["condition"] == n and c["which"] == which), None)
                row += (f" {kg(sc['mass_kg'])} kg: exposto em {fmt_heights([r['fail_skid_h_m'] for r in sc['sweep'] if r['exposed']])} |"
                        if sc else " — |")
            L.append(row)
    # 5. advisory
    L.append("\n**5. Previsão de ramos: os mesmos 130 casos do Passo 3** (mesmas massas e alturas de falha), conferidos "
             "contra a simulação completa com o avaliador corrigido.\n")
    o = p3c.get("advisory_sweep", {})
    hdr = "| Grandeza | Antigo (avaliador com erro) |" + "".join(f" {k} |" for k in vs)
    L += [hdr, "|---|---|" + "---|" * len(vs)]
    for label, ko, kn in (("Previsões corretas (os dois ramos)", "n_prediction_correct", "n_prediction_correct"),
                          ("Alertas", "n_alerts", "n_alerts"), ("Resultado mudado seguindo o alerta", "n_changed", "n_changed"),
                          ("… de inseguro para seguro", "n_unsafe_to_safe", "n_unsafe_to_safe"),
                          ("… de seguro para inseguro", "n_safe_to_unsafe", "n_safe_to_unsafe"),
                          ("Alertas errados", "n_wrong_advice", "n_wrong_advice"),
                          ("Casos perdidos (procedimento inseguro, outro seguro, sem alerta)", "n_missed", "n_missed"),
                          ("Casos sem nenhum ramo seguro", None, "n_no_branch_safe")):
        row = f"| {label} | {o.get(ko, '—') if ko else '—'} |"
        for k in vs:
            a = v[k].get("advisory", {})
            row += f" {a.get(kn, '—')} |"
        L.append(row)
    row = f"| Tempo de cálculo: média · p95 · máx. (s) | {o['wall_time_s']['mean']:.2f} · {o['wall_time_s']['p95']:.2f} · {o['wall_time_s']['max']:.2f} |"
    for k in vs:
        w = v[k].get("advisory", {}).get("wall_time_s")
        row += f" {w['mean']:.2f} · {w['p95']:.2f} · {w['max']:.2f} |" if w else " — |"
    L.append(row.replace(".", ","))
    comp["advisory"] = {k: {kk: vv for kk, vv in v[k].get("advisory", {}).items() if kk != "rows"} for k in vs}
    text = "\n".join(L) + "\n"
    with open(os.path.join(RES, "recalculo_abortar_comparacao.json"), "w", encoding="utf-8") as f:
        json.dump(comp, f, ensure_ascii=False, indent=1, default=float)
    a, z = "<!-- RECALC:BEGIN (gerado por tools/heli_reject_report.py) -->", "<!-- RECALC:END -->"
    doc = open(DOC, encoding="utf-8").read()
    if a in doc:
        i, j = doc.index(a), doc.index(z) + len(z)
        doc = doc[:i] + a + "\n" + text + z + doc[j:]
        open(DOC, "w", encoding="utf-8").write(doc)
    print(text)


if __name__ == "__main__":
    main()
