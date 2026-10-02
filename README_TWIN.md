# Twin v2: physics core

Twin v2 replaces the kinematic motion of the original Twin with a 6-DoF physics model. In that model, gravity, air density and actuator limits decide whether and how the drone flies. The code lives in `src/physics/` and depends only on `numpy` and `scipy`.

Phase 1 delivered the physics core, the thesis vehicles, the verification tests and the validation figures. Phase 2 (below) connects the web interface (`simulation_server.py`) to that core, replacing the old kinematic loop.

## Results (see `docs/twin_v2/`)

| Vehicle | Body | T/W | Hover power | Hover endurance | Status |
| --- | --- | --- | --- | --- | --- |
| `mars_hexacopter_tcc` (MSH class, 31.2 kg, 6 × 1.28 m) | Mars (0.015 kg/m³, −50 °C) | 1.31 | 6.2 kW (NASA: 6.2 kW) | 11.6 min (1200 Wh) | Flies; holds 10 m within ±0.05 m in wind |
| `tcc_mars_70kg_original` (thesis Ch. 3, 70 kg, 6 × 0.6 m) | Mars | 0.13 | — | — | Cannot take off; 6 × 0.6 m rotors lift at most ~9 kg |
| `lunar_hopper_tcc` (80 kg incl. 35 kg propellant) | Moon | 1.85 | — (propellant: 0.056 kg/s) | 13.4 min hover; Δv ≈ 1,300 m/s | Flies; holds 10 m within ±0.05 m |
| any rotorcraft | Moon | 0 | — | — | Rotors produce no thrust in vacuum |

![Feasibility](docs/twin_v2/feasibility_mars.png)
![Same mission, three vehicles](docs/twin_v2/altitude_comparison.png)

To regenerate: `python tools/twin_validation_report.py`

## Model

| Part | Model |
| --- | --- |
| Atmosphere | Constant lapse rate, hydrostatic with the body's own gravity; ρ = p/(RT); speed of sound a = √(γRT), with γ = 1.29 for CO₂ (Mars) |
| Rotor | Constant tip speed V_t = min(M_tip·a, V_motor), collective thrust control. T_max = (C_T/σ)_max·σ·ρ·A·V_t² |
| Rotor power | Momentum theory: P = T^1.5 / (FM·√(2ρA)) / η_motor; reaction torque Q = P/Ω |
| Thrusters | Throttleable, ṁ = F/(Isp·g₀) |
| Rigid body | Quaternion attitude, Newton–Euler with gyroscopic term, 4th-order Runge–Kutta at 200 Hz |
| Environment | Airframe drag relative to wind; wind = mean + Gauss–Markov gusts; spring–damper landing legs with friction |
| Control (50 Hz) | PID position loop + geometric SO(3) attitude controller (Lee et al., 2010); bounded least-squares control allocation that respects every actuator limit |

## Verification (`tests/test_physics_core.py`, 18 tests; `tests/test_web_session.py` covers Phase 2)

The tests run with `python -m pytest tests/ -v` or `python -m unittest discover -s tests -t . -v`, and in CI through `.github/workflows/twin-tests.yml`. They check:

- Free fall matches g on Earth, Mars and the Moon (to 10⁻⁶ m).
- A torque-free asymmetric tumble conserves energy and angular momentum (relative error < 10⁻⁶), and the quaternion norm stays at 1 (to 10⁻⁹).
- Rotors make zero thrust in vacuum.
- Thrust scales with ρ·V_t², so an Earth drone keeps less than 2% of its thrust on Mars.
- NASA MSH parameters reproduce NASA's 6.2 kW hover power (within 15%) and C_T/σ = 0.115.
- The thesis's original 70 kg Mars drone cannot hover, and in simulation it stays on the ground.
- The validator flags the three legacy "optimized" configurations that cannot lift off even on Earth.
- Closed-loop hover in Mars wind stays within ±0.5 m, and the time-domain power is within 5% of the closed-form estimate.
- Battery energy used equals the integrated power, and lunar propellant used equals ∫F dt/(Isp·g₀).

**Honest scope.** The MSH check confirms that the implementation reproduces NASA's numbers from NASA's own inputs. It is *verification*, not independent experimental *validation*. Values marked as assumptions in `config/drone_models.json` (stall limit C_T/σ = 0.15, battery capacity, inertia, drag area, lunar Isp ≈ 230 s) should be cited as such in the thesis.

## Usage

```python
from src.physics import load_vehicle, load_body, TwinSimulator, hover_at, hover_report

mars = load_body("mars")
hexa = load_vehicle("mars_hexacopter_tcc", payload=2.0)
print(hover_report(hexa, mars))           # closed-form T/W, power, endurance
print(hexa.validate(mars))                # physical sanity warnings

sim = TwinSimulator(hexa, mars, seed=1)
tel = sim.run(60.0, hover_at([0, 0, 10]))
print(tel.column("power_w").mean(), tel.column("battery_wh")[-1])
```

## Phase 2: the web app flies the physics core

**What changed**

- `src/physics/web_session.py` (`WebSession`) owns a `TwinSimulator`, the mission state and the telemetry dict. It has no Flask dependency and is tested in `tests/test_web_session.py`.
- `simulation_server.py` no longer moves the drone kinematically (fixed 5 m/s, fake sinusoidal attitude). At start it builds a `WebSession` from the selected vehicle, environment (or the custom environment, whose typed air density is honoured) and mission. Each 0.1 s tick then integrates 0.1 s of physics (20 RK4 steps at dt = 5 ms, control at 50 Hz) and sleeps for the rest of the tick. Pause and stop work as before.
- Missions use `waypoint_route` with each waypoint's `tolerance` as acceptance radius and `duration` as hold time. Cruise speed is the minimum of the mission's `cruise_speed` (or `constraints.max_speed`), the vehicle's `flight_envelope.max_speed` and 5 m/s. The reference now follows a trapezoidal speed profile (`max_accel` = half the tilt-limited acceleration, at most 1 m/s²). Without it, the lunar hopper (0.38 m/s² of usable horizontal acceleration) overshot waypoints by about 30 m and tipped over on landing.
- Feasibility check before flight: `vehicle.validate(body)` and `hover_report`. If T/W < 1 the vehicle is still simulated (it stays on the ground) and the server emits `twin_warning`, e.g. *"Este veículo não consegue decolar em Marte (T/W = 0,12)"*. The warning is also returned by `/api/simulation/start` and `/api/simulation/status` (`twin`, `warning`). If the vehicle has not lifted off after 20 s of simulated time, the mission ends as `failed`.
- End states: `completed` (all waypoints, `mission_complete` event), `failed` (no lift-off, tip-over > 60°, battery or propellant exhausted; `mission_failed` event), `timeout` (`success_criteria.flight_time_max`; stored as `completed_timeout` as before).
- New mission `reconhecimento_marte_tcc` ("Reconhecimento Marte - TCC"): climb to 15 m, 3 observation points within 50 m, land at the start point (about 85 s on Mars, about 105 s on the Moon). All vehicles in `config/drone_models.json`, including the three TCC vehicles, appear in the UI's vehicle list.
- UI: a compact **Física do Twin** panel (power, battery, propellant, air density, rotor tip Mach, T/W) and a warning banner. The vertical/3D speed and the Power & Energy chart now use the physics values. Older sessions without them fall back to the previous estimates.
- Performance: `np.cross` was about 70% of the step time. It is replaced by a plain 3-vector `cross3` (same arithmetic), which makes the core 2.4–3× faster. Physics uses 12–24% of each 0.1 s tick, so the web loop runs in real time at dt = 5 ms. If a slow host cannot keep up, the simulation runs slower than real time without dropping physics steps (`realtime_factor` in the status).

**Run:** `python run.py` (needs `flask`, `flask-socketio` and the database packages from `pyproject.toml`), then choose e.g. `mars_hexacopter_tcc` + `mars` + `reconhecimento_marte_tcc`.

**Telemetry** (`telemetry_update` payload): the legacy fields are unchanged (`position`, `velocity`, `attitude` roll/pitch/yaw in rad, now from the quaternion; `altitude` = height of the centre of mass, about 0.5 m when resting on the legs; `ground_speed`, `vertical_speed`, `vector_speed`, `mission_progress` in %, `current_waypoint`, `mission_status`, `total_distance`, `timestamp`). New fields:

| Field | Unit | Meaning |
| --- | --- | --- |
| `power_w` | W | Electrical power: rotors + avionics + heaters |
| `battery_wh`, `battery_pct`, `energy_consumed` | Wh, %, Wh | Battery state and energy used |
| `propellant_kg` | kg | Propellant remaining (hoppers) |
| `air_density` | kg/m³ | At the current altitude (0 in vacuum) |
| `tip_mach` | – | Rotor tip Mach number |
| `thrust_to_weight` | – | Available vertical force / current weight |
| `on_ground` | bool | A landing leg touches the ground |
| `wind_speed` | m/s | Current wind (mean + gust) |
| `angular_velocity` | rad/s | Body rates |

Note: the web app uses `config/environments.json` Mars (610 Pa, 210 K → ρ = 0.0154 kg/m³, a = 226 m/s). Under these conditions `mars_hexacopter_tcc` has T/W = 1.26 and shows a "low control margin" warning. The table above uses NASA's design point (0.015 kg/m³ at −50 °C), which gives T/W = 1.31.

## Next phases

3. Add sensor models with an EKF, a fault-injection API and residual-based fault detection (the SADPF from thesis Ch. 5.4), plus a Monte Carlo campaign.
4. Rewrite the thesis Ch. 3, 6 and 7 with these results, and clean up the repository (move videos and audio to Git LFS or Releases).
