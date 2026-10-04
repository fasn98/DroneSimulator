"""
Sweep of the autorotation flare parameters (Passo 2 adjustment 1).

Goal (set by the project owner): touchdown sink <= 1.5 m/s AND ground speed <= 15 kt, both at once.
Scenario: both engines fail at 2 s, 150 m, MTOW, ISA sea level, glide at the minimum-rate-of-descent speed.

    python -m tools.heli_flare_sweep [n_random] [seed]

Writes docs/helicoptero/flare_varredura.json (every run) and flare_varredura.png.
"""

from __future__ import annotations

import itertools
import json
import os
import random
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.physics.helicopter.procedures import KT  # noqa: E402
from src.physics.helicopter.scenarios import autorotation  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "helicoptero")
SINK_MAX = 1.5  # m/s
GS_MAX = 15 * KT  # 15 kt in m/s

# round 1 (146 runs, docs/helicoptero/flare_varredura_r1.json) showed the touchdown speed bound by the
# deceleration commanded after the flare (level_decel, fixed at 1.5 m/s^2 then); round 2 adds it.
GRID = {
    "flare_agl": [30.0, 40.0, 50.0, 60.0],  # m, flare start height
    "flare_tilt": [25.0, 30.0, 35.0],  # deg, max nose-up attitude in the flare
    "flare_decel": [4.0, 6.0, 8.0],  # m/s^2
    "flare_sink_k": [0.2, 0.3, 0.45],
    "level_decel": [1.5, 3.0, 5.0],  # m/s^2 after the flare and in the cushion
    "level_tilt": [8.0, 15.0],
    "cushion_agl": [4.0, 8.0],  # m, collective cushion start
    "cushion_tilt": [8.0, 15.0],  # deg, attitude limit during the cushion (keeps decelerating)
    "cushion_sink": [0.3, 0.8],  # m/s held by the collective in the cushion
    "nr_flare": [1.0, 1.04],  # NR reference in glide/flare (store energy up to 104 %; limit 106 %)
}


def score(r):
    land = r["landing"]
    if not land.get("landed") or r["max_nr_pct"] > 106.0:  # power-off NR limit (TCDS): an overspeed is invalid
        return 99.0
    return max(land["touchdown_sink_ms"] / SINK_MAX, land["touchdown_ground_speed_ms"] / GS_MAX)


def run(params):
    r = autorotation("forward", height=150.0, duration=50.0, **params)
    land = r["landing"]
    return {"params": params, "touchdown_sink_ms": land.get("touchdown_sink_ms"),
            "touchdown_ground_speed_kt": (land.get("touchdown_ground_speed_ms") or 0.0) / KT,
            "touchdown_pitch_up_deg": land.get("touchdown_pitch_up_deg"),
            "max_nr_pct": r["max_nr_pct"], "min_nr_before_cushion_pct": r["min_nr_before_cushion_pct"],
            "score": score(r),
            "ok": bool(land.get("landed") and land["touchdown_sink_ms"] <= SINK_MAX
                       and land["touchdown_ground_speed_ms"] <= GS_MAX and r["max_nr_pct"] <= 106.0)}


def plot():
    """Scatter of every run of both rounds; NR overspeed (> 106 %) shown apart (invalid)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    runs = []
    for name in ("flare_varredura_r1.json", "flare_varredura_r2.json"):
        path = os.path.join(OUT, name)
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                runs += json.load(f)["runs"]
    runs = [r for r in runs if r["touchdown_sink_ms"] is not None]
    valid = [r for r in runs if r["max_nr_pct"] <= 106.0]
    over = [r for r in runs if r["max_nr_pct"] > 106.0]
    best = min(valid, key=lambda r: max(r["touchdown_sink_ms"] / SINK_MAX, r["touchdown_ground_speed_kt"] / 15.0))
    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.scatter([r["touchdown_ground_speed_kt"] for r in valid], [r["touchdown_sink_ms"] for r in valid], s=14,
               alpha=.6, label=f"{len(valid)} combinações válidas")
    ax.scatter([r["touchdown_ground_speed_kt"] for r in over], [r["touchdown_sink_ms"] for r in over], s=10,
               alpha=.35, color="gray", marker="x", label=f"{len(over)} com sobrevelocidade do rotor (> 106 %)")
    ax.scatter([best["touchdown_ground_speed_kt"]], [best["touchdown_sink_ms"]], s=120, marker="*",
               color="tab:red", label="melhor (novo padrão)")
    ax.fill_between([0, 15], 0, SINK_MAX, color="tab:green", alpha=.15, label="meta (≤ 1,5 m/s e ≤ 15 kt)")
    ax.set_xlabel("velocidade no solo no toque (kt)"), ax.set_ylabel("velocidade vertical no toque (m/s)")
    ax.set_xlim(0, 45), ax.set_ylim(0, 12)
    ax.set_title("Varredura do flare na autorrotação: 2 rodadas (modelo conceitual)")
    ax.grid(alpha=.3), ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "flare_varredura.png"), dpi=120)
    return best


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--plot":
        print(plot())
        return
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 1)
    keys = list(GRID)
    full = list(itertools.product(*GRID.values()))
    sample = [dict(zip(keys, c)) for c in rng.sample(full, min(n, len(full)))]
    with Pool(2) as pool:
        res = pool.map(run, sample)
    # stage 2: one-at-a-time refinement around the best point
    best = min(res, key=lambda r: r["score"])
    seen = {json.dumps(r["params"], sort_keys=True) for r in res}
    for _ in range(2):
        cand = []
        for k in keys:
            for v in GRID[k]:
                p = dict(best["params"], **{k: v})
                if json.dumps(p, sort_keys=True) not in seen:
                    seen.add(json.dumps(p, sort_keys=True))
                    cand.append(p)
        with Pool(2) as pool:
            res += pool.map(run, cand)
        best = min(res, key=lambda r: r["score"])
    res.sort(key=lambda r: r["score"])
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "flare_varredura_r2.json"), "w", encoding="utf-8") as f:
        json.dump({"meta": {"goal": "toque <= 1,5 m/s e velocidade no solo <= 15 kt",
                            "scenario": "falha dupla a 150 m, MTOW, ISA nível do mar", "grid": GRID,
                            "n_runs": len(res), "n_ok": sum(r["ok"] for r in res)},
                   "runs": res}, f, ensure_ascii=False, indent=1)

    plot()
    for r in res[:8]:
        print(round(r["score"], 2), r["ok"], round(r["touchdown_sink_ms"] or -1, 2),
              round(r["touchdown_ground_speed_kt"], 1), round(r["touchdown_pitch_up_deg"] or 0, 1), r["params"])
    print("ok:", sum(r["ok"] for r in res), "of", len(res))


if __name__ == "__main__":
    main()
