"""
Passo 3: UTI (ICU) interior, mass and balance, fuel and radius of action of the "Helicóptero UTI" template.

Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.
O orçamento de massa e o CG NÃO substituem a pesagem e o manual de massa e balanceamento da aeronave real.

Coordinates follow the EASA TCDS R.009: station STA (mm) aft of the datum plane, which lies 2 160 mm forward of the
levelling point in the front door frame; butt line BL (mm) from the fuselage median plane, + = right.
Every item carries a status (FONTE / DERIVADO / ESTIMADO) and a source, as the parameter table does.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .isa import HeliAtmosphere
from .params import HeliParams
from .model import fuel_flow_kgs
from .rotor import best_speeds, calibrate_drag_area, fuel_flow_params, level_flight

KT = 0.514444
NM = 1852.0

SRC = {
    "airbus": "https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information",
    "brochure": "https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html",
    "tcds": "https://www.easa.europa.eu/en/downloads/7943/en",
    "easa_crew": "https://regulatorylibrary.caa.co.uk/965-2012/Content/Document%20Structure/04%20CAT/3%20AMC/"
                 "AMC2%20CAT%20POL%20MAB%20100%20d%20Mass.htm",
    "portaria2048": "https://bvsms.saude.gov.br/bvs/saudelegis/gm/2002/prt2048_05_11_2002.html",
    "hamilton_t1": "https://www.hamilton-medical.com/en_US/Products/Mechanical-ventilators/HAMILTON-T1.html",
    "zoll_x": "https://www.zoll.com/en-gb/products/defibrillators/x-series-for-hospital",
    "cfr135_209": "https://www.law.cornell.edu/cfr/text/14/135.209",
}


@dataclass
class MassItem:
    name: str
    mass_kg: float
    sta_mm: float
    bl_mm: float
    status: str  # FONTE / DERIVADO / ESTIMADO (mass); positions are always ESTIMADO
    source: str
    group: str = "equipamento"


# ------------------------------------------------------------------------------------------------
# Aircraft data
EMPTY_MASS = MassItem(
    "Massa vazia básica (MTOW 2.980 kg − carga útil 1.418 kg)", 1562.0, 4400.0, 0.0, "DERIVADO",
    f"{SRC['airbus']} (carga útil 1.418 kg); CG vazio ESTIMADO (sem dado público)", "aeronave")
FUEL_CAPACITY_KG = 560.0  # usable, standard tank (Airbus brochure; TCDS: 670–700 l usable depending on tank)
FUEL_CAPACITY_AUX_KG = 730.0  # with the auxiliary tank (Airbus brochure "with auxiliary 730kg")
FUEL_STA_MM = 4300.0  # ESTIMADO: tank under the cabin floor, close to the rotor axis
FUEL_BL_MM = 0.0

# Longitudinal C.G. envelope, EASA TCDS R.009 section 18 (EC135 T3H): straight lines between the published points
CG_FWD = ((1840.0, 4180.0), (3175.0, 4237.5))  # (mass kg, STA mm); constant 4 180 mm below 1 840 kg (assumed)
CG_AFT = ((1500.0, 4570.0), (3175.0, 4349.0))
CG_LAT_MM = 100.0  # max deviation right/left


def cg_limits(mass: float) -> tuple:
    (m1, f1), (m2, f2) = CG_FWD
    fwd = f1 if mass <= m1 else f1 + (f2 - f1) * (mass - m1) / (m2 - m1)
    (m1, a1), (m2, a2) = CG_AFT
    aft = a1 + (a2 - a1) * (mass - m1) / (m2 - m1)
    return fwd, aft


# ------------------------------------------------------------------------------------------------
# Standard UTI configuration: 1 patient, pilot, physician, nurse, equipment (Portaria GM/MS 2.048/2002 list)
CREW = [
    MassItem("Piloto (assento dianteiro direito)", 85.0, 2450.0, 350.0, "FONTE",
             f"massa padrão de tripulante técnico/de voo, 85 kg ({SRC['easa_crew']})", "tripulação"),
    MassItem("Médico (assento de cabine voltado para trás, à direita)", 85.0, 3350.0, 400.0, "FONTE",
             f"massa padrão de tripulante técnico, 85 kg ({SRC['easa_crew']})", "tripulação"),
    MassItem("Enfermeiro (assento de cabine, à direita, junto à cabeceira)", 85.0, 3950.0, 400.0, "FONTE",
             f"massa padrão de tripulante técnico, 85 kg ({SRC['easa_crew']})", "tripulação"),
]
PATIENT = MassItem("Paciente adulto (na maca, lado esquerdo)", 80.0, 4150.0, -300.0, "ESTIMADO",
                   "adulto de referência; não há massa-padrão de paciente em norma conferida", "paciente")
EQUIPMENT = [
    MassItem("Maca aeromédica com sistema de fixação e embarque (kit aeromédico)", 45.0, 4150.0, -300.0, "ESTIMADO",
             f"item exigido ({SRC['portaria2048']}); massa sem dado público de fabricante"),
    MassItem("Plataforma/trilho médico e piso da cabine UTI", 40.0, 3900.0, 0.0, "ESTIMADO",
             "estrutura do interior aeromédico; sem dado público"),
    MassItem("Ventilador mecânico de transporte (classe Hamilton-T1, só a unidade)", 6.5, 3700.0, 350.0, "FONTE",
             f"6,5 kg, unidade de ventilação ({SRC['hamilton_t1']})"),
    MassItem("Suporte do ventilador e circuito", 4.0, 3700.0, 350.0, "ESTIMADO", "sem dado público"),
    MassItem("Monitor cardioversor/desfibrilador (classe ZOLL X Series)", 5.5, 3800.0, 350.0, "FONTE",
             f"\"less than 5.5 kilograms\" — usado o limite de 5,5 kg ({SRC['zoll_x']})"),
    MassItem("Oxímetro portátil", 0.5, 3800.0, 350.0, "ESTIMADO", f"item exigido ({SRC['portaria2048']})"),
    MassItem("Bombas de infusão (2)", 4.0, 3800.0, 350.0, "ESTIMADO", f"item exigido ({SRC['portaria2048']})"),
    MassItem("Prancha longa", 7.0, 4600.0, -300.0, "ESTIMADO", f"item exigido ({SRC['portaria2048']})"),
    MassItem("Oxigênio e ar comprimido (cilindros para ≥ 2 h, reguladores, rede)", 30.0, 4700.0, 200.0, "ESTIMADO",
             f"autonomia de pelo menos 2 h exigida ({SRC['portaria2048']}); massa sem dado público"),
    MassItem("Aspirador portátil", 3.0, 3800.0, 350.0, "ESTIMADO", "sem dado público"),
    MassItem("Mochilas/kits de via aérea, acesso venoso, drenagem, parto, EPI", 20.0, 4700.0, 300.0, "ESTIMADO",
             f"materiais exigidos ({SRC['portaria2048']}); massa sem dado público"),
    MassItem("Inversor/tomadas 230 V AC e iluminação médica", 6.0, 4600.0, 0.0, "ESTIMADO", "sem dado público"),
]


@dataclass
class Loading:
    items: List[MassItem] = field(default_factory=lambda: [EMPTY_MASS, *CREW, PATIENT, *EQUIPMENT])

    @property
    def zero_fuel_mass(self) -> float:
        return sum(i.mass_kg for i in self.items)

    def cg(self, fuel_kg: float) -> Dict[str, float]:
        m = self.zero_fuel_mass + fuel_kg
        sta = (sum(i.mass_kg * i.sta_mm for i in self.items) + fuel_kg * FUEL_STA_MM) / m
        bl = (sum(i.mass_kg * i.bl_mm for i in self.items) + fuel_kg * FUEL_BL_MM) / m
        fwd, aft = cg_limits(m)
        return {"mass_kg": m, "sta_mm": sta, "bl_mm": bl, "fwd_limit_mm": fwd, "aft_limit_mm": aft,
                "margin_fwd_mm": sta - fwd, "margin_aft_mm": aft - sta, "margin_lat_mm": CG_LAT_MM - abs(bl),
                "inside": bool(fwd <= sta <= aft and abs(bl) <= CG_LAT_MM)}

    def empty_cg_range(self, fuel_states) -> tuple:
        """Range of empty-mass CG station for which every fuel state stays inside the envelope (the empty CG is
        ESTIMADO, so this shows how much it may differ)."""
        ok = []
        for x in np.arange(3900.0, 4900.0, 5.0):
            items = [MassItem(EMPTY_MASS.name, EMPTY_MASS.mass_kg, x, 0.0, "", "")] + self.items[1:]
            if all(Loading(items).cg(f)["inside"] for f in fuel_states):
                ok.append(x)
        return (min(ok), max(ok)) if ok else (None, None)


# ------------------------------------------------------------------------------------------------
RESERVE_MIN = 20.0  # RBAC 91.151(b) (ANAC, EMD 08): helicopter VFR, 20 min at normal cruise; = 14 CFR 135.209(b)
STARTUP_TAXI_KG = 8.0  # ESTIMADO: start, run-up and taxi
PER_TAKEOFF_LANDING_MIN = 3.0  # ESTIMADO: minutes at hover power for each take-off (incl. climb) and landing
FT = 0.3048
REF_ATM = HeliAtmosphere()  # reference condition for the cruise of every case: ISA ...
REF_CRUISE_ALT_M = 1000 * FT  # ... at 1 000 ft above mean sea level (decided at the Passo 3 approval)

# mission profiles: is the patient on board on the outbound / return leg?
PROFILES = {
    "resgate": (False, True),  # default: outbound empty, patient on the way back
    "transferencia": (True, False),  # patient taken to the destination, back empty
    "conservador": (True, True),  # patient on both legs
}
PROFILE_LABELS = {"resgate": "Resgate (ida sem paciente, volta com)",
                  "transferencia": "Transferência (ida com paciente, volta sem)",
                  "conservador": "Conservador (paciente nos dois trechos)"}


def fuel_flow_kg_s(p: HeliParams, power_w: float) -> float:
    return fuel_flow_kgs(p, power_w, 2)


@dataclass
class RadiusResult:
    fuel_kg: float
    radius_nm: Optional[float]
    flight_time_min: Optional[float]
    cruise_kt: Optional[float]
    reserve_kg: float
    allowances_kg: float
    takeoff_mass_kg: float
    landing_mass_kg: Optional[float]
    return_takeoff_mass_kg: Optional[float] = None
    note: str = ""


def radius_of_action(fuel_kg: float, zfm_out: float, zfm_back: Optional[float] = None,
                     p: Optional[HeliParams] = None, step_nm: float = 1.0) -> RadiusResult:
    """Out-and-back radius with the fuel on board, always flown in the reference condition (ISA, 1 000 ft MSL),
    so that the take-off condition only sets how much fuel can be loaded.

    Start/taxi; on each leg a take-off and a landing at hover power; cruise at the best-range speed of the
    current mass; the load may change at the far end (zfm_back - zfm_out, e.g. the patient boarding); final
    reserve of RESERVE_MIN at the cruising fuel flow of the final mass. No wind."""
    zfm_back = zfm_out if zfm_back is None else zfm_back
    p = p or HeliParams()
    if p.f_drag <= 0:
        p.f_drag = calibrate_drag_area(p)
    if p.ff_idle_kgh <= 0:
        p.ff_idle_kgh, p.sfc_marginal = fuel_flow_params(p)
    rho_c = REF_ATM.density(REF_CRUISE_ALT_M)
    rho_0 = REF_ATM.density(0.0)
    m0 = zfm_out + fuel_kg
    if fuel_kg <= 0:
        return RadiusResult(fuel_kg, None, None, None, 0.0, 0.0, m0, None, None, "sem combustível disponível")
    ms = np.linspace(min(zfm_out, zfm_back) - 10.0, max(zfm_out, zfm_back) + fuel_kg + 10.0, 14)
    v_tab = np.array([best_speeds(p, rho_c, m)["v_br_kt"] * KT for m in ms])
    ff_tab = np.array([fuel_flow_kg_s(p, level_flight(p, v, rho_c, m).p_engines) for v, m in zip(v_tab, ms)])
    fh_tab = np.array([fuel_flow_kg_s(p, level_flight(p, 0.0, rho_0, m).p_engines) for m in ms])

    def mission(radius_nm):
        m = m0 - STARTUP_TAXI_KG
        used = allow = STARTUP_TAXI_KG
        t = 0.0
        m_ret = None
        for leg in range(2):
            if leg == 1:
                m += zfm_back - zfm_out  # load change at the far end
                m_ret = m
            h = np.interp(m, ms, fh_tab) * PER_TAKEOFF_LANDING_MIN * 60  # take-off
            m -= h
            used += h
            allow += h
            d = 0.0
            while d < radius_nm - 1e-9:
                dd = min(step_nm, radius_nm - d)
                v = np.interp(m, ms, v_tab)
                dt = dd * NM / v
                burn = np.interp(m, ms, ff_tab) * dt
                m -= burn
                used += burn
                t += dt
                d += dd
            h = np.interp(m, ms, fh_tab) * PER_TAKEOFF_LANDING_MIN * 60  # landing
            m -= h
            used += h
            allow += h
        v = np.interp(m, ms, v_tab)
        reserve = np.interp(m, ms, ff_tab) * RESERVE_MIN * 60
        return used + reserve, float(reserve), float(allow), t, m, v, m_ret

    need0, res0, allow0, *_ = mission(0.0)
    if need0 > fuel_kg:
        return RadiusResult(fuel_kg, 0.0, 0.0, None, res0, allow0, m0, None, None,
                            "combustível não cobre partida, pousos/decolagens e reserva")
    lo, hi = 0.0, 600.0
    while hi - lo > 0.2:
        mid = 0.5 * (lo + hi)
        if mission(mid)[0] <= fuel_kg:
            lo = mid
        else:
            hi = mid
    need, reserve, allow, t, m_end, v, m_ret = mission(lo)
    return RadiusResult(fuel_kg, lo, t / 60.0, float(v) / KT, reserve, allow, m0, float(m_end), float(m_ret))


def loading_for(with_patient: bool, base: Optional[Loading] = None) -> Loading:
    base = base or Loading()
    return Loading([i for i in base.items if with_patient or i.group != "paciente"])


def catA_fuel_and_radius(cat_a_mass: float, profile: str = "resgate", p: Optional[HeliParams] = None,
                         capacity_kg: float = FUEL_CAPACITY_KG, base: Optional[Loading] = None) -> Dict[str, object]:
    """Fuel that can be loaded so that the take-off mass at the origin does not exceed the Category A mass of the
    take-off condition (and the tank), the radius it gives in the reference cruise condition, and the CG at the
    origin take-off, at the return take-off and at the end (reserve only)."""
    out_pat, back_pat = PROFILES[profile]
    l_out, l_back = loading_for(out_pat, base), loading_for(back_pat, base)
    zfm_out, zfm_back = l_out.zero_fuel_mass, l_back.zero_fuel_mass
    room = cat_a_mass - zfm_out
    fuel = float(np.clip(room, 0.0, capacity_kg))
    limited_by = "tanque" if room >= capacity_kg else ("massa Cat A" if room > 0 else "sem margem")
    r = radius_of_action(fuel, zfm_out, zfm_back, p)
    fuel_ret = (r.return_takeoff_mass_kg - zfm_back) if r.return_takeoff_mass_kg else fuel
    fuel_min = r.reserve_kg if r.radius_nm is not None else 0.0
    return {"profile": profile, "cat_a_mass_kg": cat_a_mass, "zero_fuel_mass_out_kg": zfm_out,
            "zero_fuel_mass_back_kg": zfm_back, "capacity_kg": capacity_kg, "fuel_kg": fuel,
            "fuel_limited_by": limited_by, "takeoff_mass_kg": zfm_out + fuel,
            "return_takeoff_mass_kg": r.return_takeoff_mass_kg,
            "radius_nm": r.radius_nm, "radius_km": (r.radius_nm or 0.0) * NM / 1000,
            "flight_time_min": r.flight_time_min, "cruise_kt": r.cruise_kt, "reserve_kg": r.reserve_kg,
            "allowances_kg": r.allowances_kg, "note": r.note,
            "cg_takeoff": l_out.cg(fuel), "cg_return_takeoff": l_back.cg(fuel_ret),
            "cg_reserve_only": l_back.cg(fuel_min)}
