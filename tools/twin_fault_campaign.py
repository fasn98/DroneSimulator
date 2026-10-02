"""
Twin phase 3: Monte Carlo fault campaign for the SADPF.

Every run flies the TCC Mars hexacopter (mars_hexacopter_tcc) on Mars: climb to
10 m and hover for 45 s. Wind: the operating envelope of a Mars rotorcraft (mean 5 m/s,
gust sigma 2 m/s; Ingenuity was cleared for about 10 m/s). The "nominal_storm" scenario
keeps the app's default Mars environment (15 m/s mean, 15 m/s gust sigma, dust-storm
level) to measure false alarms outside that envelope. The controller flies on the EKF estimate
(triple IMU, laser altimeter, visual navigation, barometer). One fault is injected
at a random time between 15 s and 30 s.

To avoid grading the detector on its own model ("inverse crime"), every run uses
a perturbed plant: mass x N(1, 1 %), inertia x U(0.9, 1.1) per axis, per-rotor
thrust scale x N(1, 2 %), actuator time constant x U(0.8, 1.2). The controller
and the SADPF keep the nominal model.

Output: docs/twin_v3/results.json, campaign.png, rotor_fault_example.png
Run:    python tools/twin_fault_campaign.py [--runs-scale 1.0] [--workers 2]
"""

from __future__ import annotations

import argparse
import copy
import dataclasses
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics import (RotorFault, SadpfConfig, SensorConfig, SensorFault, TwinSimulator, hover_at,  # noqa: E402
                         load_body, load_vehicle)

OUT = ROOT / "docs" / "twin_v3"
VEHICLE = "mars_hexacopter_tcc"
DURATION = 45.0
TIP_OVER_DEG = 60.0

# scenario -> number of runs (before --runs-scale)
SCENARIOS = {
    "nominal": 20,
    "nominal_storm": 10,
    "rotor_loe": 20,
    "rotor_out": 10,
    "altimeter_bias": 10,
    "altimeter_stuck": 10,
    "nav_pos_jump": 10,
    "gyro_unit_bias": 10,
    "accel_unit_bias": 10,
}
COMPARE_WITHOUT_SADPF = ("rotor_loe", "rotor_out")


def mars_body(storm: bool = False):
    body = load_body("mars")
    if storm:
        return body
    return dataclasses.replace(body, wind_mean=5.0, wind_gust_std=2.0)


def perturbed_plant(nominal, rng):
    p = copy.deepcopy(nominal)
    p.mass_dry *= rng.normal(1.0, 0.01)
    p.inertia = np.diag(np.diag(p.inertia) * rng.uniform(0.9, 1.1, 3))
    for r in p.rotors:
        r.effectiveness = rng.normal(1.0, 0.02)
        r.time_constant *= rng.uniform(0.8, 1.2)
    return p


def make_fault(scenario: str, rng, t_fault: float, n_rotors: int):
    """Returns (faults, truth) where truth describes what the SADPF should find."""
    if scenario in ("nominal", "nominal_storm"):
        return [], {}
    if scenario == "rotor_loe":
        i, eff = int(rng.integers(n_rotors)), float(rng.uniform(0.3, 0.75))
        return [("rotor", i, eff)], {"code": "rotor_loe", "rotor": i, "loe": 1 - eff}
    if scenario == "rotor_out":
        i = int(rng.integers(n_rotors))
        return [("rotor", i, 0.0)], {"code": "rotor_loe", "rotor": i, "loe": 1.0}
    if scenario == "altimeter_bias":
        b = float(rng.uniform(0.5, 3.0) * rng.choice([-1, 1]))
        return [("sensor", "altimeter", "bias", b, 0)], {"code": "sensor_altimeter", "size": b}
    if scenario == "altimeter_stuck":
        return [("sensor", "altimeter", "stuck", 0.0, 0)], {"code": "sensor_altimeter"}
    if scenario == "nav_pos_jump":
        ang, mag = rng.uniform(0, 2 * np.pi), rng.uniform(2.0, 6.0)
        return [("sensor", "nav_pos", "bias", [mag * np.cos(ang), mag * np.sin(ang), 0.0], 0)], \
            {"code": "sensor_nav_pos", "size": mag}
    if scenario == "gyro_unit_bias":
        u, axis, mag = int(rng.integers(3)), int(rng.integers(3)), float(rng.uniform(0.02, 0.1))
        v = [0.0, 0.0, 0.0]
        v[axis] = mag * rng.choice([-1, 1])
        return [("sensor", "gyro", "bias", v, u)], {"code": f"sensor_gyro#{u}", "size": mag}
    if scenario == "accel_unit_bias":
        u, axis, mag = int(rng.integers(3)), int(rng.integers(3)), float(rng.uniform(0.2, 1.0))
        v = [0.0, 0.0, 0.0]
        v[axis] = mag * rng.choice([-1, 1])
        return [("sensor", "accel", "bias", v, u)], {"code": f"sensor_accel#{u}", "size": mag}
    raise ValueError(scenario)


def build_faults(spec, t_fault):
    out = []
    for f in spec:
        if f[0] == "rotor":
            out.append(RotorFault(f[1], f[2], t_fault))
        else:
            val = np.array(f[3]) if isinstance(f[3], list) else f[3]
            out.append(SensorFault(f[1], f[2], t_fault, val, unit=f[4]))
    return out


def run_one(job):
    scenario, k, use_sadpf = job
    seed = 1000 * (list(SCENARIOS).index(scenario) + 1) + k
    rng = np.random.default_rng(seed)
    body = mars_body(storm=scenario == "nominal_storm")
    nominal = load_vehicle(VEHICLE)
    plant = perturbed_plant(nominal, rng)
    t_fault = float(rng.uniform(15.0, 30.0))
    spec, truth = make_fault(scenario, rng, t_fault, len(nominal.rotors))
    cfg = SadpfConfig() if use_sadpf else SadpfConfig(reconfigure=False, land_on_critical=False)
    sim = TwinSimulator(nominal, body, seed=seed, sensors=SensorConfig(), sadpf=cfg,
                        faults=build_faults(spec, t_fault), plant_vehicle=plant)
    tel = sim.run(DURATION, hover_at([0.0, 0.0, 10.0]))
    t = tel.column("t")
    z = tel.column("z")
    tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(tel.column("roll_deg"))) *
                                        np.cos(np.radians(tel.column("pitch_deg"))), -1, 1)))
    nominal_run = scenario in ("nominal", "nominal_storm")
    after = t >= (t_fault if not nominal_run else 15.0)
    events = [{"t": round(e.t, 3), "level": e.level, "code": e.code, "message": e.message,
               **{k2: (round(v, 4) if isinstance(v, float) else v) for k2, v in e.data.items()}}
              for e in sim.sadpf.events]
    diag = [e for e in events if e["code"] not in ("landed", "ekf_reset", "gyro_bias")]
    false_alarms = [e for e in diag if e["t"] < t_fault] if not nominal_run else diag
    hit = None
    if truth:
        for e in diag:
            if e["t"] >= t_fault and e["code"] == truth["code"] and \
                    (truth["code"] != "rotor_loe" or e.get("rotor") == truth["rotor"]):
                hit = e
                break
    # first touchdown vertical speed after a landing request
    vz = tel.column("vz")
    on_ground = tel.column("on_ground")
    td_speed = None
    if sim.sadpf.land_requested is not None:
        idx = np.where((t > sim.sadpf.land_requested) & (on_ground > 0.5))[0]
        if len(idx):
            td_speed = float(-np.min(vz[max(idx[0] - 5, 0):idx[0] + 1]))
    tipped = bool(np.max(tilt[after]) > TIP_OVER_DEG) if after.any() else False
    hold_err = float(np.max(np.abs(z[after] - 10.0))) if sim.sadpf.land_requested is None and after.any() else None
    return {
        "scenario": scenario, "run": k, "sadpf": use_sadpf, "seed": seed, "t_fault": round(t_fault, 3),
        "truth": truth, "detected": hit is not None,
        "latency_s": round(hit["t"] - t_fault, 3) if hit else None,
        "loe_estimate": hit.get("loe") if hit and "loe" in hit else None,
        "level": hit["level"] if hit else None,
        "false_alarms": len(false_alarms), "false_alarm_codes": [e["code"] for e in false_alarms],
        "landed": sim.sadpf.landed, "touchdown_speed": td_speed, "tipped_over": tipped,
        "max_tilt_after_deg": round(float(np.max(tilt[after])), 2) if after.any() else None,
        "max_alt_deviation_m": round(hold_err, 3) if hold_err is not None else None,
        "max_nav_error_m": round(float(np.max(tel.column("pos_err_m")[after])), 3) if after.any() else None,
        "events": events,
    }


def summarise(results):
    out = {}
    keys = sorted({(r["scenario"], r["sadpf"]) for r in results}, key=lambda k: (list(SCENARIOS).index(k[0]), not k[1]))
    for sc, sp in keys:
        rs = [r for r in results if r["scenario"] == sc and r["sadpf"] == sp]
        lat = [r["latency_s"] for r in rs if r["latency_s"] is not None]
        s = {"runs": len(rs), "false_alarm_runs": sum(r["false_alarms"] > 0 for r in rs),
             "tipped_over": sum(r["tipped_over"] for r in rs)}
        if not sc.startswith("nominal"):
            s.update({"detected": sum(r["detected"] for r in rs),
                      "latency_median_s": round(float(np.median(lat)), 2) if lat else None,
                      "latency_max_s": round(float(np.max(lat)), 2) if lat else None})
        if sc.startswith("rotor"):
            errs = [abs(r["loe_estimate"] - r["truth"]["loe"]) for r in rs if r["loe_estimate"] is not None]
            s["loe_abs_error_median"] = round(float(np.median(errs)), 3) if errs else None
            s["landed"] = sum(r["landed"] for r in rs)
            tds = [r["touchdown_speed"] for r in rs if r["touchdown_speed"] is not None]
            s["touchdown_speed_max"] = round(max(tds), 2) if tds else None
            devs = [r["max_alt_deviation_m"] for r in rs if r["max_alt_deviation_m"] is not None]
            s["max_alt_deviation_median_m"] = round(float(np.median(devs)), 2) if devs else None
        if sc == "nav_pos_jump":
            s["max_nav_error_median_m"] = round(float(np.median([r["max_nav_error_m"] for r in rs])), 2)
        out[f"{sc}{'' if sp else ' (sem SADPF)'}"] = s
    return out


def example_figure(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    body, v = mars_body(), load_vehicle(VEHICLE)
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for use, color, label in ((False, "#c0392b", "sem SADPF (sem reconfiguração)"), (True, "#1f6fb2", "com SADPF")):
        cfg = SadpfConfig() if use else SadpfConfig(reconfigure=False, land_on_critical=False)
        sim = TwinSimulator(v, body, seed=3, sensors=SensorConfig(), sadpf=cfg,
                            faults=[RotorFault(2, 0.4, 20.0)])
        tel = sim.run(DURATION, hover_at([0, 0, 10]))
        t = tel.column("t")
        ax[0].plot(t, tel.column("z"), color=color, label=label)
        tilt = np.hypot(tel.column("roll_deg"), tel.column("pitch_deg"))
        ax[1].plot(t, tilt, color=color)
        ax[2].plot(t, tel.column("residual_norm"), color=color)
        for e in sim.sadpf.events:
            if use:
                ax[2].axvline(e.t, color=color, ls=":", lw=1)
    for a in ax:
        a.axvline(20.0, color="k", ls="--", lw=1)
        a.grid(alpha=0.3)
    ax[0].set_ylabel("altitude (m)")
    ax[1].set_ylabel("inclinação (°)")
    ax[2].set_ylabel("|resíduo| (fração do peso)")
    ax[2].axhline(SadpfConfig().detect_threshold, color="gray", lw=1)
    ax[2].set_xlabel("tempo (s)")
    ax[0].legend(loc="lower left")
    ax[0].set_title("Hexacóptero TCC em Marte: rotor 3 perde 60% de eficácia em t = 20 s")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def campaign_figure(summary, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [k for k in summary if "sem SADPF" not in k and not k.startswith("nominal")]
    det = [summary[k]["detected"] / summary[k]["runs"] * 100 for k in names]
    lat = [summary[k]["latency_median_s"] or 0 for k in names]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].barh(names, det, color="#1f6fb2")
    ax[0].set_xlim(0, 100)
    ax[0].set_xlabel("falhas detectadas e isoladas corretamente (%)")
    ax[1].barh(names, lat, color="#e67e22")
    ax[1].set_xlabel("latência mediana de detecção (s)")
    nom = summary.get("nominal", {})
    storm = summary.get("nominal_storm", {})
    fig.suptitle(f"SADPF — Monte Carlo em Marte (planta perturbada). Alarmes falsos em voos sem falha: "
                 f"{nom.get('false_alarm_runs', 0)}/{nom.get('runs', 0)} (vento 5±2 m/s), "
                 f"{storm.get('false_alarm_runs', 0)}/{storm.get('runs', 0)} (tempestade 15±15 m/s)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs-scale", type=float, default=1.0)
    ap.add_argument("--workers", type=int, default=2)
    args = ap.parse_args()
    jobs = []
    for sc, n in SCENARIOS.items():
        n = max(1, int(round(n * args.runs_scale)))
        jobs += [(sc, k, True) for k in range(n)]
        if sc in COMPARE_WITHOUT_SADPF:
            jobs += [(sc, k, False) for k in range(n)]
    t0 = time.time()
    with Pool(args.workers) as pool:
        results = pool.map(run_one, jobs, chunksize=1)
    summary = summarise(results)
    OUT.mkdir(parents=True, exist_ok=True)
    meta = {"vehicle": VEHICLE, "body": "mars", "duration_s": DURATION, "runs": len(results),
            "wind": "mean 5 m/s, gust sigma 2 m/s (nominal_storm: app default 15 m/s, sigma 15 m/s)",
            "wall_time_s": round(time.time() - t0, 1),
            "plant_perturbation": "mass N(1,1%), inertia U(0.9,1.1), rotor thrust scale N(1,2%), tau U(0.8,1.2)",
            "sadpf": SadpfConfig().__dict__ | {"nis_reject_count": SadpfConfig().nis_reject_count}}
    with open(OUT / "results.json", "w") as f:
        json.dump({"meta": meta, "summary": summary, "runs": results}, f, indent=1, default=str)
    campaign_figure(summary, OUT / "campaign.png")
    example_figure(OUT / "rotor_fault_example.png")
    print(json.dumps(summary, indent=1, ensure_ascii=False))
    print(f"{len(results)} runs in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
