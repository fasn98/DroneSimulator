"""
"Previsão de ramos": advisory function of the SADPF (CONSULTIVO, CONCEITUAL, NÃO CERTIFICADO).

At the instant the SADPF detects an engine failure during a Category A take-off, the current state is copied and
both branches (reject and continue) are simulated faster than real time, with a coarser time step, over a short
horizon. Each predicted branch is judged with the same criteria as the scenario (procedures.evaluate_cat_a).
If the branch the procedure calls for (before / after the TDP) is predicted unsafe and the other one safe, a
highlighted advisory alert is issued. The procedure stays the default: the advisory is only displayed unless the
caller explicitly asks the "pilot" to follow it (used to measure what it would change).

The two branches run in two warm worker processes (decision of the Passo 3 approval); the wall-clock time,
including the transfer of the state, is measured and compared with the 1 s pilot reaction time (development
environment, not on-board hardware).
"""

from __future__ import annotations

import copy
import multiprocessing
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from typing import Dict, Optional

LABEL = "CONSULTIVO · conceitual · não certificado"
PRED_DT = 0.05  # s, integration step of the prediction (the scenario uses 0.005 s); control at 20 Hz
PRED_HORIZON = 25.0  # s, maximum predicted flight after the detection (the prediction stops earlier when decided)
CHUNK = 0.5  # s, the stop conditions are checked every CHUNK
NEAR_GROUND_M = 2.0  # m of skid height below which the stop conditions are checked every SHORT_CHUNK
SHORT_CHUNK = 0.1  # s
PRED_DT_GROUND = 0.01  # s, step at and near ground contact (the skids are a stiff spring-damper)
CONTACT_MARGIN_M = 0.3  # m: finer step below this skid height ...
CONTACT_LEAD_S = 0.3  # ... plus this many seconds of the current sink rate
BUDGET_S = 1.0  # pilot reaction time (procedures.PILOT_DELAY)


def _predict(sim, proc, action: str, criterion: str, vtoss_kt: Optional[float]) -> Dict[str, object]:
    from .procedures import evaluate_cat_a
    s = copy.deepcopy(sim)
    s.dt = PRED_DT
    s.ctrl_every = 1  # control at every prediction step
    s._k = 0
    pr = copy.copy(proc)
    pr.sim = s
    pr.log = list(proc.log)
    pr.advisor = None
    pr.force_action = action
    t_end = s.t + PRED_HORIZON
    t0 = s.t
    landed_t = None
    while s.t < t_end:
        agl_skid = float(s.x[2]) - s.dyn.terrain.height(s.x[0], s.x[1]) - s.p.cg_h
        v_down = max(-float(s.x[5]), 0.0)
        # stiff skid contact: the finer step only when contact is near (within ~CONTACT_LEAD_S at the current sink
        # rate) or on the ground; near the ground the stop conditions are checked more often (SHORT_CHUNK)
        near = agl_skid < CONTACT_MARGIN_M + CONTACT_LEAD_S * v_down
        if near and s.dt != PRED_DT_GROUND:
            s.dt, s.ctrl_every, s._k = PRED_DT_GROUND, int(round(0.02 / PRED_DT_GROUND)), 0
        elif not near and agl_skid > CONTACT_MARGIN_M + 0.5 and s.dt != PRED_DT:  # clear again: coarse step
            s.dt, s.ctrl_every, s._k = PRED_DT, 1, 0
        chunk = CHUNK if agl_skid > NEAR_GROUND_M + CHUNK * v_down else SHORT_CHUNK
        tel = s.run(chunk, pr, 0.1)
        x = s.x
        on_ground = s.dyn.aero(x, s.u)[3].on_ground
        if action == "abortar":
            if on_ground and s.t - t0 > 1.0:
                landed_t = landed_t or s.t
                if s.t - landed_t >= 1.0:  # settled on the skids: decided
                    break
        else:
            if on_ground and s.t - t0 > 1.0:
                break  # contact: decided (unsafe)
            v_air = float(x[3]) - float(s.dyn.wind[0])
            if vtoss_kt and v_air >= 0.95 * vtoss_kt * 0.514444 and x[5] > 0.3:
                break  # at VTOSS and climbing: the lowest point is behind
    ev = evaluate_cat_a(tel, s, vtoss_kt, "reject" if action == "abortar" else "continue", criterion)
    return {"safe": bool(ev["safe"]), "reason": ev["reason"], "predicted_s": s.t - t0}


def _timed_predict(sim, proc, action, criterion, vtoss_kt):
    t = time.perf_counter()
    r = _predict(sim, proc, action, criterion, vtoss_kt)
    r["compute_s"] = time.perf_counter() - t
    return r


_POOL = None


def _pool():
    """Two worker processes, started once and kept warm (the prediction must not pay the start-up at the
    failure). None where child processes cannot be started (e.g. inside a daemonic multiprocessing worker)."""
    global _POOL
    if _POOL is None:
        try:
            if multiprocessing.current_process().daemon or (os.cpu_count() or 1) < 2:
                _POOL = False
            else:
                _POOL = ProcessPoolExecutor(max_workers=2, mp_context=multiprocessing.get_context("fork"))
                list(_POOL.map(abs, (0, 0)))  # start and warm both workers
        except (OSError, ValueError, AssertionError, RuntimeError):
            _POOL = False
    return _POOL or None


@dataclass
class BranchPredictor:
    criterion: str = "elevado_29_60"
    vtoss_kt: Optional[float] = None
    parallel: bool = True  # the two branches in two processes (decision of the Passo 3 approval)
    result: Optional[Dict[str, object]] = None
    history: list = field(default_factory=list)

    def __post_init__(self):
        if self.parallel:
            _pool()  # warm up before the take-off, not at the failure

    def __call__(self, sim, proc, procedure_action: str) -> Dict[str, object]:
        args = (self.criterion, self.vtoss_kt)
        pool = _pool() if self.parallel else None
        t0 = time.perf_counter()
        mode = "sequencial"
        if pool is not None:
            try:  # the state is pickled and sent to both workers; the wall time includes that transfer
                fr = pool.submit(_timed_predict, sim, proc, "abortar", *args)
                fc = pool.submit(_timed_predict, sim, proc, "prosseguir", *args)
                rej, con = fr.result(), fc.result()
                mode = "paralelo"
            except Exception:  # noqa: BLE001 - a broken pool falls back to the sequential prediction
                pool = None
        if pool is None:
            rej = _timed_predict(sim, proc, "abortar", *args)
            con = _timed_predict(sim, proc, "prosseguir", *args)
        t2 = time.perf_counter()
        other = "prosseguir" if procedure_action == "abortar" else "abortar"
        pred = {"abortar": rej, "prosseguir": con}
        alert = (not pred[procedure_action]["safe"]) and pred[other]["safe"]
        each = [rej["compute_s"], con["compute_s"]]
        res = {"label": LABEL, "t": sim.t, "procedure_action": procedure_action, "predicted": pred,
               "advise": other if alert else None, "alert": alert, "mode": mode,
               "wall_time_s": t2 - t0, "wall_time_each_s": each,
               "parallel_estimate_s": max(each), "sequential_estimate_s": sum(each),
               "within_budget": (t2 - t0) <= BUDGET_S}
        self.result = res
        self.history.append(res)
        msg = (f"[{LABEL}] Previsão de ramos: o procedimento indica {procedure_action.upper()}, previsto como "
               f"inseguro ({pred[procedure_action]['reason']}); {other.upper()} previsto como seguro"
               if alert else f"[{LABEL}] Previsão de ramos: {procedure_action.upper()} previsto como "
               f"{'seguro' if pred[procedure_action]['safe'] else 'inseguro'}")
        sim.events.append({"t": sim.t, "code": "branch_prediction", "level": 2 if alert else 1,
                           "message": msg, "advisory": True, "alert": alert})
        return res
