"""
Quick check after the fix of the reject touchdown detection (aval do Passo 4): reject branch of the elevated
heliport (failure 3 m before the TDP) at the approved Category A masses and 150 / 300 / 450 kg below them.

    python -m tools.heli_reject_touchdown_check      (~5 min on 2 cores)
"""
import os
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.scenarios import CatAConfig, cat_a_run  # noqa: E402

APPROVED = {(0.0, 0.0, 0.0): 2980.0, (0.0, 20.0, 0.0): 2931.25, (1000.0, 20.0, 0.0): 2760.625,
            (1500.0, 25.0, 0.0): 2614.375, (1500.0, 25.0, 8.0): 2809.375}  # passo2b, criterion elevado_29_60


def one(a):
    cond, m = a
    r = cat_a_run(m, "reject", cfg=CatAConfig(elevation_m=cond[0], delta_t=cond[1], headwind_ms=cond[2]))
    return cond, m, r["safe"], r["reason"], r.get("touchdown_sink_ms")


if __name__ == "__main__":
    jobs = [(c, m - d) for c, m in APPROVED.items() for d in (0.0, 150.0, 300.0, 450.0)]
    with Pool(2) as p:
        for cond, m, safe, reason, sink in p.map(one, jobs):
            print(f"{cond} {m:7.1f} kg  {'seguro' if safe else 'inseguro'}  toque {sink:.2f} m/s  {reason}")
