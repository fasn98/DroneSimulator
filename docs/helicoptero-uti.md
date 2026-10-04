# Template "Helicóptero UTI": helicóptero bimotor leve de transporte aeromédico (classe H135)

> **Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.**
>
> O modelo é genérico, sem marca, logotipo ou pintura de fabricante. "Classe H135" indica apenas a ordem de grandeza da aeronave de referência, cujos dados públicos foram usados.

Estado atual: **Passo 0 e Passo 1 concluídos**. Os passos seguintes (cenários, falhas, SADPF, interior UTI e HUD) ainda não foram feitos.

## Como rodar

```bash
python -m unittest tests.test_helicopter_physics -v   # validação do Passo 1 (13 testes)
python -m unittest tests.test_template_regression      # drone de Marte idêntico ao de antes
python tools/heli_step1_report.py                      # números-chave e figura da curva de potência
python tools/heli_docs.py                              # regenera a tabela de parâmetros deste documento
```

```python
from src.physics.templates import get_template
heli = get_template("helicoptero_uti")
sim = heli.build(seed=1)                       # ISA, nível do mar, MTOW
tel = sim.run(30.0, heli.default_guidance())   # decola e paira a 15 m
```

## Arquitetura

- O helicóptero usa o mesmo motor de simulação do drone: `SimulationLoop`, com física a 200 Hz e controle a 50 Hz, o mesmo vento com *seed* e o mesmo contêiner de telemetria. Ele usa também as mesmas funções de guiagem (`hover_at`, `waypoint_route`).
- O código específico fica em `src/physics/helicopter/`:

| Arquivo | Conteúdo |
|---|---|
| `params.py` | Tabela de parâmetros com valor, unidade, status e fonte (a mesma tabela que aparece abaixo) |
| `isa.py` | Atmosfera ISA com ΔT e elevação do heliponto; altitude-densidade |
| `rotor.py` | Rotor principal, efeito solo, anel de vórtice, rotor de cauda carenado, *trim* e curva de potência |
| `model.py` | Corpo rígido 6-DoF, NR, *flapping*, dois turboeixos, esquis, limites de potência por regime |
| `simulator.py` | `HelicopterSimulator`: SAS/piloto automático, governador de NR, falhas de motor, modo autorrotação |

## Modelos e equações

### 1. Atmosfera
- ISA conforme ICAO Doc 7488 / ISO 2533.
- Na altitude h: pressão p_ISA(h) e temperatura T_ISA(h) + ΔT, com ρ = p/(R·T).
- A altitude-densidade é a altura ISA que tem a mesma ρ.
- A elevação do heliponto e o ΔT são configuráveis em `HeliAtmosphere`.

### 2. Rotor principal: elemento de pá + momento, inflow uniforme, quase-estático
- **Empuxo**, com torção linear e coletivo θ₇₅:
  C_T = (σa/2)·[θ₇₅/3·(1 + 1,5μ²) − λ/2], com λ = λ_c + λ_i.
- **Velocidade induzida**: v_i = k_GE · v_h(T) · v̂(V_c/v_h, V_x/v_h), onde v_h = √(T/2ρA).
- **Curva universal v̂**:
  - momento de Glauert, v̂·√(y² + (x+v̂)²) = 1, tomando a menor raiz positiva (ramos de funcionamento normal e de moinho);
  - na região de anel de vórtice (−2 ≤ x ≤ 0 e y < 1), mistura com peso (1 − y) da curva empírica v̂ = 1 + k₁x + k₂x² + k₃x³ + k₄x⁴ (Johnson 1980, reproduzida em Leishman 2006; coeficientes a conferir na fonte primária);
  - empuxo negativo usa a curva espelhada.
- **Solução**: por busca de raiz com intervalo (Brent). Há uma solução única e contínua em todo estado, inclusive perto de empuxo zero.
- **Potência**: P = T·(V_c + κ·v_i) + P₀·(1 + 4,65μ²), com P₀ = σ·C_d0/8·ρ·A·V_t³.
  - Na simulação 6-DoF, V_c é o escoamento através do plano das pontas. Em voo à frente, o disco inclinado contra o arrasto da fuselagem gera sozinho a parcela parasita T·V·sen α = D·V.
- **Figura de mérito resultante** (pairado, MTOW, ISA nível do mar): **0,73**.

### 3. Efeito solo (Cheeseman & Bennett, 1955)
- Com empuxo constante, a velocidade induzida é multiplicada por k_GE = 1 − (R/4z)²/(1 + (V/v_i)²).
- No pairado, isso equivale a T_IGE/T_OGE = 1/(1 − (R/4z)²) com a mesma potência; a fórmula do pairado foi conferida no PDF original.
- **Limite**: z/R ≥ **0,5**. O roteiro pedia 0,25, mas em z/R = 0,25 a fórmula é singular (razão de empuxo infinita), e os autores só relatam concordância com voo para z/R > 0,6. Esta decisão precisa do seu aval.
- No esqui (cubo a 3,0 m, z/R = 0,59), a potência de pairado cai **15 %**.

### 4. Voo à frente e curva de potência
- *Trim* em voo nivelado: T = √(W'² + D²) e inclinação do disco α = atan(D/W').
  - W' inclui o *download* da fuselagem, que some com a velocidade.
  - D = ½ρV²f.
- Potência total = induzida + perfil·(1 + 4,65μ²) + parasita ½ρV³f + rotor de cauda + acessórios, dividida pelo rendimento da transmissão.
- **Área de arrasto f: CALIBRADA**, não estimada. Escolhida de modo que o cruzeiro rápido publicado (136 kt) exija a potência máxima contínua AEO (2 × 69 % de torque = 567 kW) ao nível do mar, ISA, no MTOW. Resultado: f = **1,37 m²**.
- A velocidade de máxima autonomia é o mínimo de P. A de máximo alcance é o mínimo de P/V, sem vento.

![Curva de potência](helicoptero/curva_potencia.png)

### 5. Rotor de cauda carenado (tipo Fenestron)
- Empuxo comandado pelos pedais. O empuxo de equilíbrio anula o torque de acionamento do rotor principal: T_tr = Q/l_tr.
- Potência de ventilador carenado (razão de expansão 1): P = κ·T^1,5/√(4ρA) mais a potência de perfil.
- O resultado é **calculado**, não assumido: **11 %** da potência do rotor principal no pairado.

### 6. Rotação do rotor (NR), base da autorrotação
- I·Ω·dΩ/dt = η·(P₁ + P₂) − P_rotor − P_cauda − P_acessórios.
- A inércia I é **ESTIMADA** (1.000 kg·m², ordem de grandeza do Bo105; não há dado público do H135).
- **Governador**: a demanda de potência é a potência requerida mais um termo de erro de NR, dividida entre os motores em funcionamento e limitada por regime.
- **Proteção contra queda de NR**: o SAS devolve coletivo se o NR cair abaixo de 98,5 %.

### 7. Plano das pontas (*flapping*)
- Modelo quase estático com lag de 1ª ordem: dβ/dt = (β_cmd − β)/τ_f − q, com τ_f = 16/(γΩ) (Padfield) = **0,076 s**.
- O termo −q faz o disco "ficar no espaço" quando a fuselagem gira. Isso produz o amortecimento natural do rotor.
- Momento no cubo: braço do cubo × empuxo, mais a rigidez do rotor sem articulação (ESTIMADA).
- Não há modelo de pá elástica nem de *blowback* (ver Limitações).

### 8. Anel de vórtice (VRS)
- **Alerta** quando −1,5 ≤ V_c/v_h ≤ −0,5 e V_x/v_h < 1.
- **Fonte**: W. Johnson, NASA/TP-2005-213477, que relata movimento vertical instável entre −0,5 e −1,5·v_h (Drees 1949: −0,62 a −1,53) e efeitos de VRS que desaparecem acima de V_x ≈ v_h (ensaios de Taghizad).
- O alerta é um critério de exibição. O que acontece na física (descida acelerada) vem da curva de inflow do item 2.

### 9. Motores e transmissão
- Dois turboeixos classe PW206B3.
- **Potência térmica**: valores do TCDS IM.E.017 ao nível do mar, com lapso σ^0,75 (**ESTIMADO**).
- **Limites da transmissão**: os torques do TCDS EASA R.009 (2 × 75 % AEO TO, 2 × 69 % MCP, 128 % OEI 30 s, 125 % OEI 2 min, 86 % OEI contínuo) × potência a 100 % de torque (665 N·m a 5.898 rpm = 410,7 kW).
- O limite efetivo é o **menor** dos dois. Ao nível do mar, a transmissão limita: 616 kW AEO; 526, 513 e 353 kW OEI.
- **Regimes OEI automáticos** após a falha: 30 s → 2 min → contínuo. Esse cronômetro alimenta o HUD no Passo 4.

### 10. Comandos e SAS
- Comandos: coletivo (θ₇₅), cíclico longitudinal e lateral (inclinação comandada do disco) e pedais (empuxo do rotor de cauda).
- **SAS/piloto automático** para cenários roteirizados:
  - **horizontal**: posição → velocidade → vetor de empuxo desejado → atitude (erro em SO(3), como no drone) → momento → cíclico;
  - **vertical**: velocidade vertical → empuxo → coletivo pela relação de elemento de pá no inflow atual;
  - **guinada**: torque de acionamento + correção de proa → pedal;
  - **autorrotação**: o coletivo passa a controlar o NR, e a altura fica livre.
- O empuxo lateral do rotor de cauda é compensado por inclinação lateral: há ~4° de rolagem de equilíbrio no pairado.

## Validação (Passo 1): `tests/test_helicopter_physics.py`, 13 testes passando

| Verificação | Resultado | Critério |
|---|---|---|
| Pairado OGE, MTOW, ISA nível do mar | **594 kW** requeridos (rotor 509 + cauda 57 + acessórios, ÷ η) contra 616 kW AEO disponíveis | sem valor público direto; ver a linha seguinte |
| Teto de pairado OGE publicado: 7.200 ft ISA (dado Airbus do H135; a massa não é informada) | modelo: 639 kW requeridos contra 571 kW disponíveis a 7.200 ft, divergência de **+11,7 %** | ±15 %: **passa** (o modelo fica conservador; ver Divergências) |
| Figura de mérito no pairado | 0,73 | 0,65–0,80 |
| Efeito solo no esqui | −15 % de potência | reduz |
| Curva de potência | V máx. autonomia **69 kt** (314 kW); V máx. alcance **102 kt** (376 kW) | V_be < V_br, com balde |
| Rotor de cauda | 11 % da potência do rotor principal (calculado) | 5–20 % |
| NR sem potência e coletivo parado | cai de 100 % para 85 % em **1,5 s**; decaimento monotônico | abaixo de 85 % (mínimo sem motor do TCDS) |
| Autorrotação com coletivo baixado (modo NR) | NR entre 99,9 % e 100,0 %; descida vertical de **22,9 m/s** (≈ 1,8·v_h, como prevê a teoria) | 85–106 % (TCDS) |
| Falha de um motor | regimes OEI 30 s → 2 min seguem o cronômetro; potência disponível 526 kW | conforme TCDS |
| VRS | alerta em descida vertical a 1·v_h; sem alerta a 0,2·v_h ou com V_x = 2·v_h | critério de Johnson |
| Inflow com solução única | mesma potência partindo de chutes iniciais diferentes | ±1 W |
| Determinismo | mesma *seed* → trajetória idêntica; outra *seed* → diferente | exato |
| Decolagem, subida a 2 m/s e pairado a 15 m | NR ≥ 97 % (mínimo com motor do TCDS); deriva < 3 m | — |
| Drone de Marte (regressão) | trajetórias idênticas a 10⁻⁹ | `tests/test_template_regression.py` |

### Divergências e decisões que precisam do seu aval

1. **Teto de pairado (+11,7 %)**. A Airbus não informa a massa do teto de 7.200 ft. Se ele foi medido abaixo do MTOW, a divergência diminui. Outros parâmetros também pesam: C_d0, κ e o lapso do motor (σ^0,75) são estimados. Não ajustei nenhum deles para "fechar" o número.
2. **Diâmetro do rotor**: usei o TCDS (10,20 m, EC135). O site da Airbus informa 10,40 m para o H135. Com 10,40 m, a potência de pairado cai ~4 %.
3. **Mistura de variantes**: o MTOW e o desempenho são do H135 (Airbus). Os limites de torque, NR e motor são do TCDS EC135 P2/P3 e do PW206B3, porque os blocos T3H/P3H do TCDS não foram extraídos. As classificações do Arrius 2B2Plus (H135 T3) não foram encontradas.
4. **Limite do efeito solo em z/R ≥ 0,5** em vez de 0,25 (ver item 3 dos modelos).
5. **OEI contínuo**: o TCDS do PW206B3 não lista esse regime. Usei a máxima contínua térmica, e o limite efetivo acaba sendo o da transmissão (86 % de torque).

## Limitações conhecidas (não implementadas)
- **Rotor**: inflow uniforme, sem pá elástica, sem *blowback* e sem acoplamentos de *flapping* com a velocidade. A estabilidade estática em voo à frente vem só do SAS.
- **Aerodinâmica**: compressibilidade e estol de pá recuante não são modelados, então o modelo não prevê V_NE. Faltam a sustentação da fuselagem, a deriva e o estabilizador horizontal.
- **Rotor de cauda**: sem descarga em voo à frente (o empuxo da deriva não é modelado).
- **Motores**: modelo de 1ª ordem com consumo específico estimado (sem dado público). Não há modelo de Ng.
- **Pouso**: esquis mola-amortecedor simples. Pouso com flare e taxa de toque são assunto do Passo 2.

## Parâmetros (status: FONTE / DERIVADO / CALIBRADO / ESTIMADO)

<!-- PARAMS:BEGIN (gerado por tools/heli_docs.py) -->
| Parâmetro | Valor | Unid. | Status | Fonte / justificativa |
|---|---|---|---|---|
| Massa máxima de decolagem (H135, configuração interna) | 2.980 | kg | **FONTE** | [link](https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information) |
| Combustível, tanque padrão | 560 | kg | **FONTE** | [link](https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html) |
| Raio do rotor principal (TCDS EC135: Ø 10,20 m; o site Airbus do H135 informa Ø 10,40 m) | 5,1 | m | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Número de pás (rotor sem articulação, bearingless) | 4 | - | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Corda equivalente da pá (Kampa et al., ERF 1997) | 0,288 | m | **FONTE** | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| Rotação do rotor a 100 % NR | 395 | rpm | **FONTE** | [link](https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download) |
| Velocidade de ponta publicada | 211 | m/s | **FONTE** | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| NR mínimo com motor (EC135 P2/P3) | 97 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| NR máximo com motor (EC135 P2/P3) | 104 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| NR mínimo sem motor, massa > 1.900 kg (EC135 P1/P2) | 85 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| NR máximo sem motor (EC135 P1/P2) | 106 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Inclinação da curva de sustentação | 5,73 | 1/rad | **ESTIMADO** | valor típico de perfil de pá (≈ 0,91·2π) |
| Arrasto de perfil médio da pá | 0,01 | - | **ESTIMADO** | faixa típica 0,008–0,012 para pás de rotor |
| Fator de potência induzida κ | 1,15 | - | **ESTIMADO** | fator típico de perdas induzidas (Leishman, Principles of Helicopter Aerodynamics) |
| Fator de perfil em voo à frente | 4,65 | - | **ESTIMADO** | fator (1 + 4,65 μ²) da potência de perfil em voo à frente (literatura clássica) |
| Inércia polar do rotor principal | 1.000 | kg m² | **ESTIMADO** | ordem de grandeza do Bo105 (4 pás × ~232 kg m², Padfield), sem dado público do H135 |
| Número de Lock γ (constante de tempo do flapeamento 16/(γΩ)) | 5,1 | - | **ESTIMADO** | ordem de grandeza do Bo105 (Padfield), sem dado público do H135 |
| Rigidez de momento do cubo | 130.000 | N m/rad | **ESTIMADO** | rotor sem articulação: offset equivalente ~10 %, pá ~30 kg |
| Altura do cubo acima do CG | 1,7 | m | **ESTIMADO** | geometria típica da classe |
| Raio do rotor de cauda carenado (Ø 1,00 m) | 0,5 | m | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Pás do rotor de cauda carenado | 10 | - | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Corda da pá do rotor de cauda | 0,05 | m | **FONTE** | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| Velocidade de ponta do rotor de cauda | 188 | m/s | **FONTE** | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| Braço do rotor de cauda | 6 | m | **ESTIMADO** | distância eixo do rotor principal → rotor de cauda, geometria da classe |
| Empuxo máximo do rotor de cauda (100 % NR, nível do mar) | 4.500 | N | **ESTIMADO** | margem ~2× o empuxo de equilíbrio em pairado no MTOW |
| Potência a 100 % de torque, por motor | 410,7 | kW | **DERIVADO** | 665 N·m × 5.898 rpm ([link](https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download)) |
| Torque de decolagem, dois motores (2 × 75 %) | 75 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Torque máximo contínuo, dois motores (2 × 69 %) | 69 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Torque OEI 30 s (1 × 128 %) | 128 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Torque OEI 2 min (1 × 125 %) | 125 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Torque OEI contínuo (1 × 86 %) | 86 | % | **FONTE** | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Turboeixo classe PW206B3: decolagem (nível do mar) | 336 | kW | **FONTE** | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Turboeixo: máximo contínuo | 324 | kW | **FONTE** | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Turboeixo: OEI 30 s | 547 | kW | **FONTE** | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Turboeixo: OEI 2 min | 534 | kW | **FONTE** | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Expoente de lapso do motor | 0,75 | - | **ESTIMADO** | potência térmica ∝ σ^0,75 com a altitude-densidade |
| Constante de tempo do motor | 0,8 | s | **ESTIMADO** | resposta típica de turboeixo pequeno |
| Constante de tempo da queda de potência | 1 | s | **ESTIMADO** | desaceleração após apagamento |
| Rendimento da transmissão | 0,97 | - | **ESTIMADO** | valor típico de caixa principal |
| Potência de acessórios | 10 | kW | **ESTIMADO** | geradores, bombas, ar-condicionado médico |
| Consumo específico | 0,36 | kg/kWh | **ESTIMADO** | consumo específico típico de turboeixo da classe (sem dado público) |
| Download | 0,03 | - | **ESTIMADO** | arrasto vertical da fuselagem na esteira, típico 2–5 % |
| Área de arrasto lateral | 5 | m² | **ESTIMADO** | área lateral da fuselagem × Cd |
| Área de arrasto vertical | 6 | m² | **ESTIMADO** | área em planta da fuselagem × Cd |
| Amortecimento de guinada | 4.000 | N m s/rad | **ESTIMADO** | amortecimento de guinada do rotor de cauda e da deriva |
| Limite de inclinação do disco | 10 | ° | **ESTIMADO** | curso típico do cíclico em inclinação do plano das pontas |
| Curso do coletivo (θ75 máx.) | 18 | ° | **ESTIMADO** | θ75 de -2° a 16° |
| Inércia de rolagem | 1.200 | kg m² | **ESTIMADO** | classe 3 t |
| Inércia de arfagem | 4.500 | kg m² | **ESTIMADO** | classe 3 t |
| Inércia de guinada | 3.800 | kg m² | **ESTIMADO** | classe 3 t |
| Meia bitola do esqui | 1,1 | m | **ESTIMADO** | geometria da classe |
| Meio comprimento do esqui | 1,3 | m | **ESTIMADO** | geometria da classe |
| Altura do CG sobre o esqui | 1,3 | m | **ESTIMADO** | geometria da classe |
<!-- PARAMS:END -->

Além da tabela:
- a **área de arrasto f = 1,37 m²** é CALIBRADA (item 4);
- os **coeficientes da curva empírica de VRS** vêm da literatura (Johnson 1980 / Leishman 2006), a conferir na fonte primária;
- os **ganhos do SAS e do governador** são de projeto do controlador, não da aeronave.
