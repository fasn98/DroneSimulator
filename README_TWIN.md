# Twin v2: physics core

Twin v2 replaces the kinematic motion of the original Twin with a 6-DoF physics model. In that model, gravity, air density and actuator limits decide whether and how the drone flies. The code lives in `src/physics/` and depends only on `numpy` and `scipy`.

Phase 1 (this branch) delivers the physics core, the thesis vehicles, the verification tests and the validation figures. The web interface (`simulation_server.py`) still uses the old kinematic loop. Connecting it is Phase 2.

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

## Verification (`tests/test_physics_core.py`, 18 tests)

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

## Next phases

2. Connect `simulation_server.py` to this core and add charts for power, battery and tip Mach to the web interface.
3. Add sensor models with an EKF, a fault-injection API and residual-based fault detection (the SADPF from thesis Ch. 5.4), plus a Monte Carlo campaign.
4. Rewrite the thesis Ch. 3, 6 and 7 with these results, and clean up the repository (move videos and audio to Git LFS or Releases).
