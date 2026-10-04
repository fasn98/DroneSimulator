"""
SADPF for the helicopter: engine-failure detection, isolation and recommended action.

Same levels and event format as the drone SADPF (src/physics/sadpf.py): 1 Aviso, 2 Alerta, 3 Crítico.

Detection uses only what the cockpit measures (no knowledge of the injected fault):
    engine power (torque x NR, with sensor noise), governor demand, rotor speed.
    single failure:  torque split  (P_hi - P_lo) / P_hi > SPLIT  for PERSIST s    -> lower engine failed
                     (in AEO the governor commands both engines equally, so a split is an engine fault)
    dual failure:    total power < DUAL_FRAC x demand for PERSIST s, with NR decaying
                     (or, after a single failure, the remaining engine below DUAL_FRAC of its demand)
Recommended action, computed from the physics at the detection instant:
    dual                     -> AUTORROTAÇÃO
    single, Cat A before TDP -> ABORTAR (pouso no heliponto)    single, Cat A after TDP -> PROSSEGUIR
    single, other phases     -> OEI 2-min power available vs power required at the current speed and at
                                the minimum-power speed: PROSSEGUIR EM OEI (margin) or POUSO IMEDIATO
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from ..sadpf import LEVEL_NAMES
from .model import engine_limit_w
from .rotor import best_speeds, level_flight

SPLIT = 0.35
DUAL_FRAC = 0.40
PERSIST = 0.15  # s
NOISE = 0.01  # 1-sigma relative torque-sensor noise (ESTIMADO)


@dataclass
class HeliSadpf:
    rng: np.random.Generator
    level: int = 0
    events: List[dict] = field(default_factory=list)
    failed: List[bool] = field(default_factory=lambda: [False, False])
    dual: bool = False
    recommendation: str = ""
    recommendation_code: str = ""
    detect_t: Optional[float] = None
    _p_f: np.ndarray = field(default_factory=lambda: np.zeros(2))
    _split_t: float = 0.0
    _dual_t: float = 0.0
    _init: bool = False

    def _event(self, sim, level: int, code: str, message: str, **data):
        self.level = max(self.level, level)
        ev = {"t": sim.t, "level": level, "level_name": LEVEL_NAMES[level], "code": code, "message": message,
              "data": data}
        self.events.append(ev)
        sim.events.append(ev)

    def _recommend(self, sim, dual: bool):
        p = sim.p
        x = sim.x
        if dual:
            return "autorrotacao", "AUTORROTAÇÃO: baixar o coletivo, manter a velocidade de mínima razão de descida, flare perto do solo"
        phase = getattr(sim, "flight_phase", None)
        if phase == "catA_pre_tdp":
            return "abortar", "ABORTAR: falha antes do TDP — pousar no heliponto com potência OEI"
        if phase == "catA_post_tdp":
            return "prosseguir", "PROSSEGUIR: falha após o TDP — baixar o nariz, acelerar até VTOSS com OEI 30 s"
        rho = sim.atm.density(x[2])
        m = sim.dyn.mass(x)
        p_av = engine_limit_w(p, sim.atm, x[2], "OEI2", 1)
        v_now = float(np.linalg.norm(x[3:6] - sim.dyn.wind))
        p_now = level_flight(p, v_now, rho, m).p_engines
        bs = best_speeds(p, rho, m)
        p_min = bs["p_be_kw"] * 1e3
        if p_av >= p_min:
            return ("prosseguir_oei", f"PROSSEGUIR EM OEI: acelerar para ~{bs['v_be_kt']:.0f} kt "
                                      f"(margem OEI 2 min {100 * (p_av / p_min - 1):.0f} % na velocidade de mínima potência; "
                                      f"{100 * (p_av / p_now - 1):+.0f} % na velocidade atual)")
        return "pouso_imediato", (f"POUSO IMEDIATO: potência OEI {p_av / 1e3:.0f} kW < mínima requerida "
                                  f"{p_min / 1e3:.0f} kW")

    def update(self, sim, dt: float):
        x = sim.x
        meas = np.array([x[16], x[17]]) * (1.0 + NOISE * self.rng.standard_normal(2))
        if not self._init:
            self._p_f = meas.copy()
            self._init = True
        a = dt / (0.1 + dt)
        self._p_f += a * (meas - self._p_f)
        pf = np.maximum(self._p_f, 0.0)
        demand = float(sum(sim.u.p_cmd))
        nr = sim.x[13] / sim.p.omega100
        running = [i for i in range(2) if not self.failed[i]]
        # single failure: torque split while both believed running
        if len(running) == 2 and max(pf) > 50e3:
            hi, lo = (0, 1) if pf[0] >= pf[1] else (1, 0)
            split = (pf[hi] - pf[lo]) / pf[hi]
            self._split_t = self._split_t + dt if split > SPLIT else 0.0
            if self._split_t >= PERSIST:
                self.failed[lo] = True
                self.detect_t = sim.t
                code, msg = self._recommend(sim, dual=False)
                self.recommendation_code, self.recommendation = code, msg
                self._event(sim, 3, "engine_failure", f"Falha do motor {lo + 1} detectada (divergência de torque "
                            f"{100 * split:.0f} %). {msg}", engine=lo + 1, action=code)
        # dual failure
        total = float(sum(pf[i] for i in running))
        ref = demand if len(running) == 2 else float(sim.u.p_cmd[running[0]]) if running else 0.0
        low = ref > 100e3 and total < DUAL_FRAC * ref and nr < 0.99
        if running and low:
            self._dual_t += dt
        else:
            self._dual_t = 0.0
        if running and self._dual_t >= PERSIST:
            for i in running:
                self.failed[i] = True
            self.dual = True
            self.detect_t = sim.t
            code, msg = self._recommend(sim, dual=True)
            self.recommendation_code, self.recommendation = code, msg
            self._event(sim, 3, "dual_engine_failure", f"Perda de potência nos dois motores. {msg}", action=code)
