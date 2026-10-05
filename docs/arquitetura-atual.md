# Arquitetura atual do Gêmeo Digital (antes do template "Helicóptero UTI")

Este documento descreve o repositório como ele está no commit `c401cb4` (fase 3 do Twin). Ele também registra a abstração `VehicleTemplate`, introduzida para receber um segundo tipo de veículo sem duplicar o motor de simulação.

## Stack

| Camada | Tecnologia | Onde |
|---|---|---|
| Núcleo de física | Python 3, `numpy` + `scipy`, sem outras dependências | `src/physics/` |
| Servidor | Flask + Flask-SocketIO; cadência em tempo real e telemetria por WebSocket | `simulation_server.py` |
| Sessão web | `WebSession`: missão, telemetria em dicionário JSON estrito e injeção de falhas; testável sem Flask | `src/physics/web_session.py` |
| Front-end / HUD | HTML + JS puro (Three.js para a cena 3D), HUD em português | `web/index.html`, `web/script.js`, `web/style.css`; helicóptero UTI: `web/heli/` (rota `/heli/`) |
| Configuração | Veículos, ambientes (Terra, Marte, Lua) e missões em JSON | `config/drone_models.json`, `config/environments.json`, `config/missions.json` |
| Testes | `unittest` (também rodam com pytest); CI no GitHub Actions: suíte completa e teste de fumaça do HUD (ver `README_TWIN.md`) | `tests/`, `.github/workflows/twin-tests.yml` |

Os módulos antigos da raiz e de `src/` (`physics_engine.py`, `drone_controller.py`, vídeos e marketing) são da versão cinemática. O Twin v2/v3 não depende deles.

## Loop de física

- **Corpo rígido 6-DoF** (`dynamics.py`):
  - estado: posição e velocidade (ENU), atitude em quatérnio, velocidade angular no corpo, força de cada atuador (lag de 1ª ordem), bateria em Wh e propelente em kg;
  - integração por Runge–Kutta de 4ª ordem com **dt = 5 ms (200 Hz)**.
- **Atmosfera** (`atmosphere.py`):
  - `Body` com gravidade, pressão e temperatura de superfície, gradiente térmico, R e γ;
  - ρ = p/(RT) e a = √(γRT), com a gravidade do próprio corpo na equação hidrostática.
- **Atuadores** (`actuators.py`):
  - rotor de velocidade de ponta constante, empuxo por coletivo com T_max = (C_T/σ)_max·σ·ρ·A·V_t²;
  - potência P = T^1,5/(FM·√(2ρA)) e torque de reação Q = P/Ω;
  - propulsores químicos.
- **Ambiente**:
  - arrasto da fuselagem relativo ao vento;
  - vento médio + rajadas Gauss–Markov (`WindModel`);
  - pernas de pouso mola-amortecedor com atrito.

## Controle (50 Hz)

`control.py`:

- malha de posição PID e controle geométrico de atitude em SO(3) (Lee et al., 2010);
- alocação por mínimos quadrados limitados (`lsq_linear`), que respeita o limite de cada atuador;
- o vetor `Allocator.effectiveness` permite a reconfiguração após uma falha.

## Sensores, navegação e SADPF (fase 3)

- **Sensores** (`sensors.py`): três IMUs com votação pela mediana, altímetro laser, navegação visual e barômetro (ruído em Pa convertido pela densidade local).
- **EKF** (`ekf.py`): filtro de estado de erro com 15 estados (Solà 2017), *gating* por NIS χ² e recuperação de divergência.
- **SADPF** (`sadpf.py`):
  - detecção por resíduos de força e momento filtrados;
  - isolamento do rotor pela coluna da matriz de controle que melhor explica o resíduo, com estimativa da perda de eficiência (LOE);
  - níveis 1 (Aviso), 2 (Alerta) e 3 (Crítico);
  - prognóstico da razão empuxo/peso pós-falha por programação linear, com pouso autônomo se T/W < 1,1;
  - detector de pouso.

## Cenários e falhas

- **Missões**: listas de waypoints em `config/missions.json`, seguidas por `waypoint_route` (perfil de velocidade trapezoidal).
- **Falhas**: `RotorFault(index, effectiveness, t_start)` e `SensorFault(sensor, kind, t_start, ...)`, injetadas no `TwinSimulator` ou pela rota `POST /api/simulation/fault`. Os eventos do SADPF chegam ao HUD pelo evento `sadpf_event`.
- **Campanha Monte Carlo**: `tools/twin_fault_campaign.py`, com 140 voos e planta perturbada (resultados em `docs/twin_v3/`).

## HUD

`web/index.html` + `web/script.js`:

- cena 3D, painel de telemetria (altitude, velocidades, potência, bateria, densidade do ar, Mach de ponta, T/W);
- painel SADPF (nível, eventos, eficiência por rotor) e botões de injeção de falha;
- gráficos de física e legenda inferior.

## Abstração `VehicleTemplate` (introduzida no Passo 0)

**Objetivo:** trocar entre "Drone Marte" e "Helicóptero UTI" sem duplicar o motor.

| Parte | Comum a todos os veículos (`simloop.py`) | Específica do template |
|---|---|---|
| Relógio, passo fixo, laço `run`, cadência de gravação | `SimulationLoop` | — |
| Vento e rajadas, gerador aleatório com *seed* | `WindModel` | — |
| Contêiner de telemetria | `Telemetry` | campos gravados (`telemetry_fields`) |
| Planta (dinâmica) | — | multirrotor (`dynamics.py`) ou helicóptero (`helicopter/`) |
| Controle | — | controle geométrico do drone ou SAS/piloto automático do helicóptero |
| Ambiente padrão | — | `default_body()` (Marte ou Terra ISA) |

- `src/physics/templates.py` define `VehicleTemplate`, com `build(body, seed, **opções)`, `default_body()`, `default_guidance()`, `telemetry_fields` e `label`, mais o registro `get_template(id)` e `list_templates()`.
- O template `drone_marte` apenas envolve o `TwinSimulator`. O `TwinSimulator` passou a herdar `SimulationLoop`, e seu `run` é o mesmo código, movido sem alteração.
- **Regressão**: `tools/make_regression_golden.py` gravou, **antes** da refatoração, duas trajetórias do drone com *seed* fixa:
  - rota com vento;
  - pairado com SADPF e falha de rotor.

  O teste `tests/test_template_regression.py` exige que o código refatorado as reproduza com tolerância de 10⁻⁹. O teste passa, ou seja, o drone voa de forma idêntica.
