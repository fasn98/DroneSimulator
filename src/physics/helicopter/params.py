"""
Parameters of the "classe H135" light twin EMS helicopter (generic, no manufacturer branding).

Every number carries its status:
    FONTE      read from a public, citable document (URL in `source`)
    DERIVADO   computed from FONTE values (formula in `source`)
    CALIBRADO  fitted so the model reproduces a published performance figure
    ESTIMADO   engineering estimate, no public source found (to be replaced by real data)

The same table is exported to docs/helicoptero-uti.md and docs/fontes.md
(tools/heli_docs.py), so code and documentation cannot drift apart.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict

# --- source URLs ------------------------------------------------------------------------------
SRC = {
    "airbus": "https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information",
    "airbus_brochure": "https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html",
    "tcds_r009": "https://www.easa.europa.eu/en/downloads/7943/en",
    "tcds_r009_05": "https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-"
                    "EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf",
    "tcds_pw206": "https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962",
    "kampa1997": "https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download",
    "doleschel2007": "https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download",
    "johnson_vrs": "https://ntrs.nasa.gov/api/citations/20060024029/downloads/20060024029.pdf",
    "cheeseman": "https://reports.aerade.cranfield.ac.uk/bitstream/handle/1826.2/3590/arc-rm-3021.pdf?sequence=1&isAllowed=y",
}


@dataclass(frozen=True)
class P:
    """One parameter: value, unit, status and where it comes from."""
    value: float
    unit: str
    status: str
    source: str
    label: str
    variant: str  # which aircraft / document the number belongs to (shown in docs and HUD)


VARIANT_RULES = (
    ("airbus", "H135 (dados Airbus)"),
    ("7943", "EC135 T3/P3/T3H/P3H, TCDS EASA R.009"),
    ("R.009", "EC135 P2/P3, TCDS EASA R.009"),
    ("caa.co.uk", "PW206B3 (motor do EC135 P3), TCDS IM.E.017"),
    ("8af39742", "EC135 (Kampa et al., 1997)"),
    ("02b3ce46", "EC135 (Doleschel & Emmerling, 2007)"),
)


def _variant(source: str, status: str) -> str:
    for key, name in VARIANT_RULES:
        if key in source:
            return name
    return "classe H135 (estimativa do modelo)" if status in ("ESTIMADO", "CALIBRADO") else "derivado"


def _p(value, unit, status, source, label, variant: str | None = None) -> P:
    assert status in ("FONTE", "DERIVADO", "CALIBRADO", "ESTIMADO"), status
    return P(float(value), unit, status, source, label, variant or _variant(source, status))


# 100 % engine torque (Doleschel & Emmerling, ERF 2007): 665 N m at the gearbox input, 5,898 rpm
TORQUE_100_NM = 665.0
GEARBOX_INPUT_RPM = 5898.0
P100_KW = TORQUE_100_NM * GEARBOX_INPUT_RPM * 2 * math.pi / 60 / 1000  # 410.7 kW per engine at 100 % torque

TABLE: Dict[str, P] = {
    # mass and fuel
    "mtow": _p(2980, "kg", "FONTE", SRC["airbus"], "Massa máxima de decolagem (H135, configuração interna)"),
    "fuel_capacity": _p(560, "kg", "FONTE", SRC["airbus_brochure"], "Combustível, tanque padrão"),
    # main rotor
    "rotor_radius": _p(5.10, "m", "FONTE", SRC["tcds_r009"],
                       "Raio do rotor principal (TCDS: Ø 10,20 m para EC135 T3, P3, T3H e P3H; o site Airbus do H135 "
                       "informa Ø 10,40 m, não confirmado no TCDS)"),
    "n_blades": _p(4, "-", "FONTE", SRC["tcds_r009"], "Número de pás (rotor sem articulação, bearingless)"),
    "chord": _p(0.288, "m", "FONTE", SRC["kampa1997"], "Corda equivalente da pá (Kampa et al., ERF 1997)"),
    "rpm_100": _p(395, "rpm", "FONTE", SRC["doleschel2007"], "Rotação do rotor a 100 % NR"),
    "tip_speed_ref": _p(211, "m/s", "FONTE", SRC["kampa1997"], "Velocidade de ponta publicada"),
    "nr_power_on_min": _p(97, "%", "FONTE", SRC["tcds_r009_05"], "NR mínimo com motor (EC135 P2/P3)"),
    "nr_power_on_max": _p(104, "%", "FONTE", SRC["tcds_r009_05"], "NR máximo com motor (EC135 P2/P3)"),
    "nr_power_off_min": _p(85, "%", "FONTE", SRC["tcds_r009_05"], "NR mínimo sem motor, massa > 1.900 kg (EC135 P1/P2)", "EC135 P1/P2, TCDS EASA R.009"),
    "nr_power_off_max": _p(106, "%", "FONTE", SRC["tcds_r009_05"], "NR máximo sem motor (EC135 P1/P2)", "EC135 P1/P2, TCDS EASA R.009"),
    "lift_slope": _p(5.73, "1/rad", "ESTIMADO", "valor típico de perfil de pá (≈ 0,91·2π)", "Inclinação da curva de sustentação"),
    "cd0": _p(0.010, "-", "ESTIMADO", "faixa típica 0,008–0,012 para pás de rotor", "Arrasto de perfil médio da pá"),
    "kappa": _p(1.15, "-", "ESTIMADO", "fator típico de perdas induzidas (Leishman, Principles of Helicopter Aerodynamics)",
                "Fator de potência induzida κ"),
    "k_mu": _p(4.65, "-", "ESTIMADO", "fator (1 + 4,65 μ²) da potência de perfil em voo à frente (literatura clássica)",
               "Fator de perfil em voo à frente"),
    "rotor_inertia": _p(1000, "kg m²", "ESTIMADO",
                        "ordem de grandeza do Bo105 (4 pás × ~232 kg m², Padfield), sem dado público do H135",
                        "Inércia polar do rotor principal"),
    "lock_number": _p(5.1, "-", "ESTIMADO", "ordem de grandeza do Bo105 (Padfield), sem dado público do H135",
                      "Número de Lock γ (constante de tempo do flapeamento 16/(γΩ))"),
    "hub_stiffness": _p(1.3e5, "N m/rad", "ESTIMADO", "rotor sem articulação: offset equivalente ~10 %, pá ~30 kg",
                        "Rigidez de momento do cubo"),
    "hub_height": _p(1.7, "m", "ESTIMADO", "geometria típica da classe", "Altura do cubo acima do CG"),
    # tail rotor (Fenestron-type ducted fan)
    "tr_radius": _p(0.50, "m", "FONTE", SRC["tcds_r009"], "Raio do rotor de cauda carenado (Ø 1,00 m)"),
    "tr_blades": _p(10, "-", "FONTE", SRC["tcds_r009"], "Pás do rotor de cauda carenado"),
    "tr_chord": _p(0.05, "m", "FONTE", SRC["kampa1997"], "Corda da pá do rotor de cauda"),
    "tr_tip_speed": _p(188, "m/s", "FONTE", SRC["kampa1997"], "Velocidade de ponta do rotor de cauda"),
    "tr_arm": _p(6.0, "m", "ESTIMADO", "distância eixo do rotor principal → rotor de cauda, geometria da classe",
                 "Braço do rotor de cauda"),
    "tr_thrust_max": _p(4500, "N", "ESTIMADO", "margem ~2× o empuxo de equilíbrio em pairado no MTOW",
                        "Empuxo máximo do rotor de cauda (100 % NR, nível do mar)"),
    "tr_expansion": _p(1.0, "-", "ESTIMADO",
                       "teoria do ventilador carenado ideal (Leishman): P = κ·T^1,5/√(4·σd·ρ·A); com σd = 1 o duto "
                       "carrega metade do empuxo; mesma tração → potência induzida 1/√2 da de um rotor aberto; mesma potência → empuxo 2^(1/3) ≈ 1,26×; sem dado público "
                       "do difusor do Fenestron", "Razão de expansão do duto do Fenestron σd"),
    "fin_area": _p(0.9, "m²", "FONTE", SRC["kampa1997"], "Área da deriva vertical (\"small fin\", configuração "
                   "básica VFR do EC135 em 1997; a deriva do H135 atual pode diferir)"),
    "fin_lift_slope": _p(3.0, "1/rad", "ESTIMADO", "superfície de baixo alongamento (~1,5), ordem de grandeza "
                         "de 2πA/(2+A)", "Inclinação da curva de sustentação da deriva"),
    "fin_incidence": _p(6.0, "°", "ESTIMADO", "incidência/arqueamento efetivo da deriva; escolhido para a deriva "
                        "assumir cerca de metade do antitorque no cruzeiro rápido (o princípio de projeto da deriva "
                        "arqueada que alivia o Fenestron é público, o valor não)", "Incidência efetiva da deriva"),
    "fin_cl_max": _p(1.0, "-", "ESTIMADO", "estol da deriva de baixo alongamento", "CL máximo da deriva"),
    # engines and transmission (per engine; power limits from torque limits x 100 % torque power)
    "p100": _p(P100_KW, "kW", "DERIVADO", f"665 N·m × 5.898 rpm ({SRC['doleschel2007']})",
               "Potência a 100 % de torque, por motor", "EC135 (Doleschel & Emmerling, 2007)"),
    "tq_aeo_to": _p(75, "%", "FONTE", SRC["tcds_r009_05"], "Torque de decolagem, dois motores (2 × 75 %)"),
    "tq_aeo_mcp": _p(69, "%", "FONTE", SRC["tcds_r009_05"], "Torque máximo contínuo, dois motores (2 × 69 %)"),
    "tq_oei_30s": _p(128, "%", "FONTE", SRC["tcds_r009_05"], "Torque OEI 30 s (1 × 128 %)"),
    "tq_oei_2min": _p(125, "%", "FONTE", SRC["tcds_r009_05"], "Torque OEI 2 min (1 × 125 %)"),
    "tq_oei_mcp": _p(86, "%", "FONTE", SRC["tcds_r009_05"], "Torque OEI contínuo (1 × 86 %)"),
    "eng_to": _p(336, "kW", "FONTE", SRC["tcds_pw206"], "Turboeixo classe PW206B3: decolagem (nível do mar)"),
    "eng_mcp": _p(324, "kW", "FONTE", SRC["tcds_pw206"], "Turboeixo: máximo contínuo"),
    "eng_oei_30s": _p(547, "kW", "FONTE", SRC["tcds_pw206"], "Turboeixo: OEI 30 s"),
    "eng_oei_2min": _p(534, "kW", "FONTE", SRC["tcds_pw206"], "Turboeixo: OEI 2 min"),
    "eng_lapse_exp": _p(0.75, "-", "ESTIMADO", "potência térmica ∝ σ^0,75 com a altitude-densidade",
                        "Expoente de lapso do motor"),
    "eng_tau": _p(0.8, "s", "ESTIMADO", "resposta típica de turboeixo pequeno", "Constante de tempo do motor"),
    "eng_fail_tau": _p(1.0, "s", "ESTIMADO", "desaceleração após apagamento", "Constante de tempo da queda de potência"),
    "transmission_eff": _p(0.97, "-", "ESTIMADO", "valor típico de caixa principal", "Rendimento da transmissão"),
    "accessory_kw": _p(10, "kW", "ESTIMADO", "geradores, bombas, ar-condicionado médico", "Potência de acessórios"),
    "sfc": _p(0.36, "kg/kWh", "ESTIMADO", "consumo específico típico de turboeixo da classe (sem dado público)",
              "Consumo específico"),
    # airframe
    "download_frac": _p(0.03, "-", "ESTIMADO", "arrasto vertical da fuselagem na esteira, típico 2–5 %", "Download"),
    "f_side": _p(5.0, "m²", "ESTIMADO", "área lateral da fuselagem × Cd", "Área de arrasto lateral"),
    "f_vertical": _p(6.0, "m²", "ESTIMADO", "área em planta da fuselagem × Cd", "Área de arrasto vertical"),
    "yaw_damping": _p(4000, "N m s/rad", "ESTIMADO", "amortecimento de guinada do rotor de cauda e da deriva",
                      "Amortecimento de guinada"),
    "cyclic_limit": _p(10, "°", "ESTIMADO", "curso típico do cíclico em inclinação do plano das pontas",
                       "Limite de inclinação do disco"),
    "collective_range": _p(18, "°", "ESTIMADO", "θ75 de -2° a 16°", "Curso do coletivo (θ75 máx.)"),
    "ixx": _p(1200, "kg m²", "ESTIMADO", "classe 3 t", "Inércia de rolagem"),
    "iyy": _p(4500, "kg m²", "ESTIMADO", "classe 3 t", "Inércia de arfagem"),
    "izz": _p(3800, "kg m²", "ESTIMADO", "classe 3 t", "Inércia de guinada"),
    "skid_half_track": _p(1.1, "m", "ESTIMADO", "geometria da classe", "Meia bitola do esqui"),
    "skid_half_length": _p(1.3, "m", "ESTIMADO", "geometria da classe", "Meio comprimento do esqui"),
    "cg_height": _p(1.3, "m", "ESTIMADO", "geometria da classe", "Altura do CG sobre o esqui"),
}


def v(key: str) -> float:
    return TABLE[key].value


@dataclass
class HeliParams:
    """Numbers the physics uses (plain floats), built from TABLE plus the calibrated drag area."""
    mass: float = field(default_factory=lambda: v("mtow"))
    fuel: float = field(default_factory=lambda: v("fuel_capacity"))
    R: float = field(default_factory=lambda: v("rotor_radius"))
    nb: int = field(default_factory=lambda: int(v("n_blades")))
    chord: float = field(default_factory=lambda: v("chord"))
    omega100: float = field(default_factory=lambda: v("rpm_100") * 2 * math.pi / 60)
    a: float = field(default_factory=lambda: v("lift_slope"))
    cd0: float = field(default_factory=lambda: v("cd0"))
    kappa: float = field(default_factory=lambda: v("kappa"))
    k_mu: float = field(default_factory=lambda: v("k_mu"))
    I_rotor: float = field(default_factory=lambda: v("rotor_inertia"))
    lock: float = field(default_factory=lambda: v("lock_number"))
    hub_k: float = field(default_factory=lambda: v("hub_stiffness"))
    hub_h: float = field(default_factory=lambda: v("hub_height"))
    tr_R: float = field(default_factory=lambda: v("tr_radius"))
    tr_sigma: float = field(default_factory=lambda: v("tr_blades") * v("tr_chord") / (math.pi * v("tr_radius")))
    tr_vt: float = field(default_factory=lambda: v("tr_tip_speed"))
    tr_arm: float = field(default_factory=lambda: v("tr_arm"))
    tr_tmax: float = field(default_factory=lambda: v("tr_thrust_max"))
    tr_sigma_d: float = field(default_factory=lambda: v("tr_expansion"))
    fin_S: float = field(default_factory=lambda: v("fin_area"))
    fin_a: float = field(default_factory=lambda: v("fin_lift_slope"))
    fin_alpha0: float = field(default_factory=lambda: math.radians(v("fin_incidence")))
    fin_clmax: float = field(default_factory=lambda: v("fin_cl_max"))
    p100_kw: float = P100_KW
    eta_tr: float = field(default_factory=lambda: v("transmission_eff"))
    p_acc_kw: float = field(default_factory=lambda: v("accessory_kw"))
    sfc: float = field(default_factory=lambda: v("sfc"))
    eng_tau: float = field(default_factory=lambda: v("eng_tau"))
    eng_fail_tau: float = field(default_factory=lambda: v("eng_fail_tau"))
    lapse_exp: float = field(default_factory=lambda: v("eng_lapse_exp"))
    download: float = field(default_factory=lambda: v("download_frac"))
    f_side: float = field(default_factory=lambda: v("f_side"))
    f_vertical: float = field(default_factory=lambda: v("f_vertical"))
    yaw_damping: float = field(default_factory=lambda: v("yaw_damping"))
    cyclic_limit: float = field(default_factory=lambda: math.radians(v("cyclic_limit")))
    theta_min: float = math.radians(-2.0)
    theta_max: float = field(default_factory=lambda: math.radians(v("collective_range") - 2.0))
    inertia: tuple = field(default_factory=lambda: (v("ixx"), v("iyy"), v("izz")))
    skid_w: float = field(default_factory=lambda: v("skid_half_track"))
    skid_l: float = field(default_factory=lambda: v("skid_half_length"))
    cg_h: float = field(default_factory=lambda: v("cg_height"))
    f_drag: float = 0.0  # equivalent flat-plate area, m^2 (CALIBRADO in rotor.calibrate_drag_area)
    rotor_dir: int = -1  # main rotor turns clockwise seen from above (model convention)

    @property
    def area(self) -> float:
        return math.pi * self.R ** 2

    @property
    def sigma(self) -> float:
        return self.nb * self.chord / (math.pi * self.R)

    @property
    def tip_speed(self) -> float:
        return self.omega100 * self.R

    @property
    def tau_flap(self) -> float:
        """Flapping time constant 16/(gamma Omega) (Padfield, Helicopter Flight Dynamics)."""
        return 16.0 / (self.lock * self.omega100)

    # power limits at 100 % NR (kW, delivered to the gearbox)
    def gearbox_limit_kw(self, rating: str) -> float:
        tq = {"TO": 2 * v("tq_aeo_to"), "MCP": 2 * v("tq_aeo_mcp"), "OEI30": v("tq_oei_30s"),
              "OEI2": v("tq_oei_2min"), "OEIC": v("tq_oei_mcp")}[rating]
        return tq / 100.0 * self.p100_kw

    def engine_rating_kw(self, rating: str) -> float:
        """Thermodynamic engine rating at sea level ISA, per engine (OEI continuous: MCP used, not published)."""
        return {"TO": v("eng_to"), "MCP": v("eng_mcp"), "OEI30": v("eng_oei_30s"), "OEI2": v("eng_oei_2min"),
                "OEIC": v("eng_mcp")}[rating]
