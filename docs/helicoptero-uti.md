# Template "Helicóptero UTI": helicóptero bimotor leve de transporte aeromédico (classe H135)

> **Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.**
>
> O modelo é genérico, sem marca, logotipo ou pintura de fabricante. "Classe H135" indica apenas a ordem de grandeza da aeronave de referência, cujos dados públicos foram usados.

Estado atual: **Passos 0 e 1 aprovados; Passo 2 (cenários, falhas e SADPF) concluído e aguardando aval.** Os passos seguintes (interior UTI, HUD e documentação final) ainda não foram feitos.

## Como rodar

```bash
python -m unittest tests.test_helicopter_physics -v   # validação do Passo 1 (13 testes)
python -m unittest tests.test_helicopter_scenarios -v # validação do Passo 2 (11 testes, ~4 min)
python -m unittest tests.test_template_regression      # drone de Marte idêntico ao de antes
python tools/heli_step1_report.py                      # números-chave e figura da curva de potência
python -m tools.heli_step2_report                      # cenários do Passo 2 (~11 min em 2 núcleos)
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
| `sadpf.py` | SADPF do helicóptero: detecção e isolamento de falha de motor e ação recomendada (Passo 2) |
| `procedures.py` | "Piloto" dos cenários: Categoria A, autorrotação com flare, rota ponto a ponto, critérios (Passo 2) |
| `scenarios.py` | Cenários prontos e cálculo da massa máxima Categoria A (Passo 2) |

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
- **Limite (aprovado)**: a fórmula é usada para z/R ≥ **0,5**. Abaixo disso o fator fica **constante no valor de z/R = 0,5** (não é zerado nem extrapolado). Motivo: em z/R = 0,25 a fórmula é singular (razão de empuxo infinita), e os autores só relatam concordância com voo para z/R > 0,6.
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

### Decisões do Passo 1 (aprovadas)

1. **Teto de pairado (+11,7 %, modelo pessimista)**: aprovado sem ajuste e registrado como limitação conhecida (abaixo). A Airbus não informa a massa do teto de 7.200 ft; se ele foi medido abaixo do MTOW, a divergência diminui. C_d0, κ e o lapso do motor (σ^0,75) são estimados, e nenhum deles foi ajustado para "fechar" o número.
2. **Diâmetro do rotor: 10,20 m**. Conferido no TCDS EASA R.009 (Issue 20, dez. 2025): o diâmetro é 10,20 m para as variantes EC135 T3, P3, T3H e P3H, as que correspondem ao H135. O valor de 10,40 m do site da Airbus **não** aparece no TCDS, então foi mantido 10,20 m, e a validação do Passo 1 continua valendo sem reprocessamento. Com 10,40 m a potência de pairado cairia ~4 %.
3. **Variantes misturadas**: aprovado. A procedência de cada parâmetro está na coluna "Variante / documento" da tabela de parâmetros e no resumo abaixo. O mesmo rótulo será exibido no HUD (Passo 4).
4. **Efeito solo**: ver item 3 dos modelos.
5. **OEI contínuo**: o TCDS do PW206B3 não lista esse regime. Usei a máxima contínua térmica, e o limite efetivo acaba sendo o da transmissão (86 % de torque).
6. **Dados do fabricante**: o modelo usa somente dados públicos. Dados de desempenho do manual de voo (RFM), como as massas Categoria A certificadas, só poderiam ser usados com **autorização formal do fabricante**.

### Procedência por variante (resumo)

| Variante / documento | O que vem dela |
|---|---|
| H135 (dados Airbus) | MTOW 2.980 kg, combustível 560 kg; referências de validação: cruzeiro rápido 136 kt e teto de pairado OGE 7.200 ft |
| EC135 T3/P3/T3H/P3H, TCDS EASA R.009 (Issue 20) | diâmetro 10,20 m, 4 pás, rotor de cauda carenado Ø 1,00 m com 10 pás |
| EC135 P2/P3, TCDS EASA R.009 (Issue 05) | limites de torque AEO e OEI, NR com motor (97–104 %) |
| EC135 P1/P2, TCDS EASA R.009 (Issue 05) | NR sem motor (85–106 %) |
| PW206B3 (motor do EC135 P3), TCDS IM.E.017 | potências térmicas por regime |
| EC135 (Kampa et al. 1997; Doleschel & Emmerling 2007) | corda, velocidades de ponta, rpm, torque a 100 % |
| classe H135 (estimativa do modelo) | todos os itens ESTIMADO e CALIBRADO |

## Passo 2: cenários, falhas e SADPF

Tudo o que segue é **calculado pela física**. Nenhum cenário tem a falha ou a reação "marcada" no tempo:
- a falha é injetada por uma condição (altura, fase do voo);
- o SADPF a detecta só pelo que a cabine mede;
- o "piloto" (`procedures.py`) segue a ação que o SADPF recomendou, depois de um tempo de reação.

O governador também não sabe da falha injetada: ele só exclui um motor depois que o SADPF o declara em falha.

### SADPF do helicóptero (`sadpf.py`)

| Item | Regra | Status |
|---|---|---|
| Medidas | potência de cada motor (torque × NR) com ruído de 1 % (1σ), demanda do governador, NR | ruído ESTIMADO |
| Falha de um motor | divergência de torque (P_alto − P_baixo)/P_alto > 35 % por 0,15 s; o motor de menor torque é o isolado | limiares ESTIMADOS |
| Falha dupla | potência total < 40 % da demanda, com NR < 99 %, por 0,15 s | limiares ESTIMADOS |
| Nível | 3 (Crítico), mesmo formato de evento do SADPF do drone | — |
| Ação recomendada | falha dupla → **AUTORROTAÇÃO**; Categoria A antes do TDP → **ABORTAR**; depois do TDP → **PROSSEGUIR**; outras fases → compara a potência OEI 2 min com a potência mínima requerida: **PROSSEGUIR EM OEI** (mostra a margem) ou **POUSO IMEDIATO** | DERIVADO da física |
| Tempo de reação do piloto após o alerta | 1,0 s | ESTIMADO |

Tempos de detecção medidos: **~0,7 s** para falha de um motor e **~1,7–2,0 s** para falha dupla (o critério de falha dupla espera o NR começar a cair).

### Cenário 1: transferência inter-hospitalar
- Percurso: decolagem, subida, cruzeiro a 300 m e 110 kt, aproximação em rampa de 8°, pairado e pouso vertical. Vento de 5 m/s com rajadas.
- Resultado em 20 km: voo de **7 min**, consumo de **24 kg** (consumo específico ESTIMADO), toque a 1,0 m/s, erro de posição de 2,9 m.
- O SADPF fica no nível 0, sem alarme falso.

### Cenário 2: resgate em área restrita
- Rampa de 12°, pairado baixo em efeito solo e **vento cruzado de 8 m/s** com rajadas de 1,5 m/s.
- No pairado final: fator de efeito solo **0,97**, potência de **439 kW** contra **537 kW** fora do efeito solo com o mesmo vento.
- Pedal máximo de 43 %, rolagem máxima de 3,7°, toque a 0,5 m/s, erro de posição de 3,0 m.

### Cenário 3: Categoria A em heliponto elevado (DEMO)

**Configuração** (`CatAConfig`):
- deck de 20 m × 20 m a 30 m acima da rua (ESTIMADO);
- TDP a 12 m de altura dos esquis acima do deck (ESTIMADO, configurável);
- elevação, ΔT ISA e vento de proa configuráveis.

**Procedimento modelado** (não é o procedimento do RFM):
- subida vertical até o TDP e depois aceleração;
- falha **reconhecida antes do TDP**: **abortar** e pousar no deck com potência OEI;
- falha **após o TDP**: **prosseguir**, baixando o nariz e trocando altura por velocidade até a VTOSS, depois subindo na VTOSS;
- os regimes OEI seguem o cronômetro 30 s → 2 min.

**Critérios de segurança do modelo**:

| Ramo | Critério | Status |
|---|---|---|
| Abortar | pousa no deck (a 1 m da borda), velocidade de toque ≤ 1,5 m/s, sem tombamento | ESTIMADO |
| Prosseguir | sem contato com o deck ou o solo | — |
| Prosseguir | atinge a VTOSS | — |
| Prosseguir | razão de subida OEI 2 min na VTOSS, fora do efeito solo, ≥ **100 ft/min** | [14 CFR 29.67(a)(1)](https://www.ecfr.gov/current/title-14/section-29.67) |
| Prosseguir | esquis ≥ **15 ft** acima do deck enquanto sobre ele e até 10 m além da borda | [14 CFR 29.59(c)](https://www.ecfr.gov/current/title-14/section-29.59) (o texto vale para toda a decolagem continuada; aqui só perto do deck) |
| Prosseguir | depois disso, ≥ 35 ft acima da rua | ESTIMADO, por analogia com a separação de obstáculos de 35 ft; não conferido em texto oficial |

**VTOSS**: a menor velocidade (≥ 25 kt) em que a subida OEI 2 min atinge 100 ft/min, pelo método de energia (DERIVADO). O piso de 25 kt é ESTIMADO.

**Massa máxima Categoria A**: a maior massa em que os **dois** ramos são seguros. É calculada por bisseção em massa (tolerância de 25 kg), com uma falha logo antes do TDP e outra logo depois:

| Heliponto | ΔT ISA | Vento de proa | Massa máx. Cat A | Limitada por |
|---|---|---|---|---|
| nível do mar | 0 | 0 | **2.980 kg** | MTOW |
| nível do mar | +20 °C | 0 | **2.931 kg** | prosseguir |
| 1.000 m | +20 °C | 0 | **2.761 kg** | prosseguir |
| 1.500 m | +25 °C | 0 | **2.614 kg** | prosseguir |
| 1.500 m | +25 °C | 8 m/s | **2.785 kg** | prosseguir |

- O vento de proa aumenta a massa admissível, como esperado.
- Nesses casos o limite é sempre o ramo **prosseguir**. Abortar continua seguro até o MTOW enquanto a aeronave consegue chegar ao TDP.
- Com 1.000 m ISA+20 ou 1.500 m ISA+25 no MTOW, a aeronave **nem chega ao TDP** com os dois motores: falta potência para pairar fora do efeito solo.
- Esses valores são do **modelo**. Como o teto de pairado é pessimista em 11,7 %, as massas em altitude tendem a ficar abaixo das reais.

**Demonstração do limite** (1.500 m, ISA+25, sem vento):

| | No limite (2.614 kg) | Acima do limite (2.700 kg) |
|---|---|---|
| Abortar | seguro: toque a 0,17 m/s no deck | seguro: toque a 0,12 m/s |
| Prosseguir | **seguro**: mínimo de 5,3 m acima do deck perto da borda; desce 6,6 m abaixo do nível do deck já longe dele; perda máxima de altura de **21,8 m** | **inseguro**: passa 5,1 m **abaixo** do nível do deck a menos de 10 m da borda; perda de altura de **36,8 m**; passa a 6,2 m da rua |
| Margem OEI 30 s no pairado OGE | −80 kW (não paira com um motor: precisa trocar altura por velocidade) | −103 kW |
| Margem OEI 2 min na VTOSS | +15,9 kW (VTOSS 25 kt) | +15,5 kW (VTOSS 28 kt) |
| Detecção pelo SADPF | 0,69 s, recomenda ABORTAR / PROSSEGUIR conforme o TDP | igual |

![Categoria A](helicoptero/categoria_a.png)

Para o HUD (Passo 4), cada execução fornece:
- massa máxima Cat A da configuração;
- ramo e ação recomendada;
- cronômetro OEI (`rating` e tempo desde a detecção);
- margens de potência OEI;
- perda máxima de altura;
- se o ramo foi seguro e por quê.

### Cenário 4: autorrotação (falha dupla)

Os dois motores param em t = 2 s, a 300 m, MTOW, ISA. O piloto só reage depois do alerta do SADPF e do tempo de reação:
- baixa o coletivo (o coletivo passa a controlar o NR);
- **à frente**: mantém a velocidade de mínima razão de descida e faz o flare;
- **vertical**: sem velocidade, só o amortecimento final com o coletivo.

| | À frente (≈ 70 kt) | Vertical |
|---|---|---|
| Velocidade de mínima razão de descida (método de energia, DERIVADO) | 70 kt | — |
| Razão de descida estabilizada, simulada | **9,7 m/s** | **22,6 m/s** |
| Razão de descida prevista | 10,4 m/s (energia: W·V_z = P_rotor + P_cauda + P_acess.) | 19,9 m/s (momento ideal); a teoria da curva empírica dá ≈ 1,7–1,8·v_h |
| NR antes do amortecimento | 94,9–104,8 % (limites sem motor do TCDS: 85–106 %) | 94,1–102,0 % |
| Toque | **1,4 m/s** vertical, **14,3 m/s** (28 kt) de velocidade no solo: pouso corrido | **19,6 m/s**: impacto |

- A comparação mostra por que a autorrotação se faz com velocidade à frente: a razão de descida cai para menos da metade, e a energia cinética permite o flare.
- A vertical a partir de 300 m termina em impacto. Esse é o comportamento esperado da região a evitar do diagrama altura-velocidade; o diagrama H-V completo (opcional no roteiro) não foi gerado.
- **Flare** (piloto do modelo, parâmetros ESTIMADOS e ajustados no modelo):
  - começa a 45 m;
  - desaceleração de até 4 m/s², modulada pela razão de descida (alvo de 0,3·(h − 6 m), entre 1,5 e 6 m/s);
  - atitude máxima de 25° nariz acima;
  - amortecimento com o coletivo abaixo de ~6 m.
- **Limitação**: o pouso termina corrido a ~28 kt. Varrendo os parâmetros do flare, este "piloto" simples consegue toque suave **ou** baixa velocidade no solo, mas não os dois ao mesmo tempo. Um flare melhor (por exemplo, otimizado ou com cíclico e coletivo coordenados) fica como melhoria.

![Autorrotação](helicoptero/autorrotacao.png)

### Validação (Passo 2): `tests/test_helicopter_scenarios.py`, 11 testes

| Teste | Critério |
|---|---|
| Sem alarme falso em voo normal | SADPF no nível 0; pousa a < 5 m do destino |
| Falha de um motor em cruzeiro | nível 3; isola o motor certo; detecção < 1,5 s; recomenda PROSSEGUIR EM OEI |
| Categoria A leve (2.500 kg, nível do mar) | falha antes do TDP → ABORTAR, seguro; falha depois → PROSSEGUIR, seguro, usa OEI 30 s |
| Limite de massa (1.500 m, ISA+25) | 2.550 kg prosseguir seguro; 2.700 kg inseguro, com maior perda de altura; abortar seguro; no MTOW não atinge o TDP |
| VTOSS | existe uma velocidade que atinge 100 ft/min |
| Determinismo | mesma *seed* → mesma trajetória |
| Autorrotação à frente | razão de descida ±15 % do método de energia; NR entre 85 e 106 % antes do amortecimento; toque ≤ 2,5 m/s e ≤ 15 m/s no solo |
| Vertical × à frente | razão de descida > 1,8× e toque > 3× mais forte |
| Velocidade de mínima razão de descida | entre 55 e 85 kt |
| Resgate | pousa a < 5 m; toque ≤ 1,5 m/s; fator de efeito solo < 1; potência menor que fora do efeito solo |

Números completos em `docs/helicoptero/passo2_resultados.json` (gerado por `tools/heli_step2_report.py`).

## Limitações conhecidas (não implementadas)
- **Teto de pairado OGE pessimista em 11,7 %** em relação ao valor publicado de 7.200 ft (decisão 1 acima). Isso também deixa conservadoras as massas Categoria A em altitude.
- **Rotor**: inflow uniforme, sem pá elástica, sem *blowback* e sem acoplamentos de *flapping* com a velocidade. A estabilidade estática em voo à frente vem só do SAS.
- **Aerodinâmica**: compressibilidade e estol de pá recuante não são modelados, então o modelo não prevê V_NE. Faltam a sustentação da fuselagem, a deriva e o estabilizador horizontal.
- **Rotor de cauda**: sem descarga em voo à frente (o empuxo da deriva não é modelado).
- **Motores**: modelo de 1ª ordem com consumo específico estimado (sem dado público). Não há modelo de Ng.
- **Pouso**: esquis mola-amortecedor simples, sem modelo de dano. Os limites de toque usados nos critérios são ESTIMADOS.

## Parâmetros (status: FONTE / DERIVADO / CALIBRADO / ESTIMADO)

<!-- PARAMS:BEGIN (gerado por tools/heli_docs.py) -->
| Parâmetro | Valor | Unid. | Status | Variante / documento | Fonte / justificativa |
|---|---|---|---|---|---|
| Massa máxima de decolagem (H135, configuração interna) | 2.980 | kg | **FONTE** | H135 (dados Airbus) | [link](https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information) |
| Combustível, tanque padrão | 560 | kg | **FONTE** | H135 (dados Airbus) | [link](https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html) |
| Raio do rotor principal (TCDS: Ø 10,20 m para EC135 T3, P3, T3H e P3H; o site Airbus do H135 informa Ø 10,40 m, não confirmado no TCDS) | 5,1 | m | **FONTE** | EC135 T3/P3/T3H/P3H, TCDS EASA R.009 | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Número de pás (rotor sem articulação, bearingless) | 4 | - | **FONTE** | EC135 T3/P3/T3H/P3H, TCDS EASA R.009 | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Corda equivalente da pá (Kampa et al., ERF 1997) | 0,288 | m | **FONTE** | EC135 (Kampa et al., 1997) | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| Rotação do rotor a 100 % NR | 395 | rpm | **FONTE** | EC135 (Doleschel & Emmerling, 2007) | [link](https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download) |
| Velocidade de ponta publicada | 211 | m/s | **FONTE** | EC135 (Kampa et al., 1997) | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| NR mínimo com motor (EC135 P2/P3) | 97 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| NR máximo com motor (EC135 P2/P3) | 104 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| NR mínimo sem motor, massa > 1.900 kg (EC135 P1/P2) | 85 | % | **FONTE** | EC135 P1/P2, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| NR máximo sem motor (EC135 P1/P2) | 106 | % | **FONTE** | EC135 P1/P2, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| Inclinação da curva de sustentação | 5,73 | 1/rad | **ESTIMADO** | classe H135 (estimativa do modelo) | valor típico de perfil de pá (≈ 0,91·2π) |
| Arrasto de perfil médio da pá | 0,01 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | faixa típica 0,008–0,012 para pás de rotor |
| Fator de potência induzida κ | 1,15 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | fator típico de perdas induzidas (Leishman, Principles of Helicopter Aerodynamics) |
| Fator de perfil em voo à frente | 4,65 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | fator (1 + 4,65 μ²) da potência de perfil em voo à frente (literatura clássica) |
| Inércia polar do rotor principal | 1.000 | kg m² | **ESTIMADO** | classe H135 (estimativa do modelo) | ordem de grandeza do Bo105 (4 pás × ~232 kg m², Padfield), sem dado público do H135 |
| Número de Lock γ (constante de tempo do flapeamento 16/(γΩ)) | 5,1 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | ordem de grandeza do Bo105 (Padfield), sem dado público do H135 |
| Rigidez de momento do cubo | 130.000 | N m/rad | **ESTIMADO** | classe H135 (estimativa do modelo) | rotor sem articulação: offset equivalente ~10 %, pá ~30 kg |
| Altura do cubo acima do CG | 1,7 | m | **ESTIMADO** | classe H135 (estimativa do modelo) | geometria típica da classe |
| Raio do rotor de cauda carenado (Ø 1,00 m) | 0,5 | m | **FONTE** | EC135 T3/P3/T3H/P3H, TCDS EASA R.009 | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Pás do rotor de cauda carenado | 10 | - | **FONTE** | EC135 T3/P3/T3H/P3H, TCDS EASA R.009 | [link](https://www.easa.europa.eu/en/downloads/7943/en) |
| Corda da pá do rotor de cauda | 0,05 | m | **FONTE** | EC135 (Kampa et al., 1997) | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| Velocidade de ponta do rotor de cauda | 188 | m/s | **FONTE** | EC135 (Kampa et al., 1997) | [link](https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download) |
| Braço do rotor de cauda | 6 | m | **ESTIMADO** | classe H135 (estimativa do modelo) | distância eixo do rotor principal → rotor de cauda, geometria da classe |
| Empuxo máximo do rotor de cauda (100 % NR, nível do mar) | 4.500 | N | **ESTIMADO** | classe H135 (estimativa do modelo) | margem ~2× o empuxo de equilíbrio em pairado no MTOW |
| Potência a 100 % de torque, por motor | 410,7 | kW | **DERIVADO** | EC135 (Doleschel & Emmerling, 2007) | 665 N·m × 5.898 rpm ([link](https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download)) |
| Torque de decolagem, dois motores (2 × 75 %) | 75 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| Torque máximo contínuo, dois motores (2 × 69 %) | 69 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| Torque OEI 30 s (1 × 128 %) | 128 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| Torque OEI 2 min (1 × 125 %) | 125 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| Torque OEI contínuo (1 × 86 %) | 86 | % | **FONTE** | EC135 P2/P3, TCDS EASA R.009 | [link](https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf) |
| Turboeixo classe PW206B3: decolagem (nível do mar) | 336 | kW | **FONTE** | PW206B3 (motor do EC135 P3), TCDS IM.E.017 | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Turboeixo: máximo contínuo | 324 | kW | **FONTE** | PW206B3 (motor do EC135 P3), TCDS IM.E.017 | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Turboeixo: OEI 30 s | 547 | kW | **FONTE** | PW206B3 (motor do EC135 P3), TCDS IM.E.017 | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Turboeixo: OEI 2 min | 534 | kW | **FONTE** | PW206B3 (motor do EC135 P3), TCDS IM.E.017 | [link](https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962) |
| Expoente de lapso do motor | 0,75 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | potência térmica ∝ σ^0,75 com a altitude-densidade |
| Constante de tempo do motor | 0,8 | s | **ESTIMADO** | classe H135 (estimativa do modelo) | resposta típica de turboeixo pequeno |
| Constante de tempo da queda de potência | 1 | s | **ESTIMADO** | classe H135 (estimativa do modelo) | desaceleração após apagamento |
| Rendimento da transmissão | 0,97 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | valor típico de caixa principal |
| Potência de acessórios | 10 | kW | **ESTIMADO** | classe H135 (estimativa do modelo) | geradores, bombas, ar-condicionado médico |
| Consumo específico | 0,36 | kg/kWh | **ESTIMADO** | classe H135 (estimativa do modelo) | consumo específico típico de turboeixo da classe (sem dado público) |
| Download | 0,03 | - | **ESTIMADO** | classe H135 (estimativa do modelo) | arrasto vertical da fuselagem na esteira, típico 2–5 % |
| Área de arrasto lateral | 5 | m² | **ESTIMADO** | classe H135 (estimativa do modelo) | área lateral da fuselagem × Cd |
| Área de arrasto vertical | 6 | m² | **ESTIMADO** | classe H135 (estimativa do modelo) | área em planta da fuselagem × Cd |
| Amortecimento de guinada | 4.000 | N m s/rad | **ESTIMADO** | classe H135 (estimativa do modelo) | amortecimento de guinada do rotor de cauda e da deriva |
| Limite de inclinação do disco | 10 | ° | **ESTIMADO** | classe H135 (estimativa do modelo) | curso típico do cíclico em inclinação do plano das pontas |
| Curso do coletivo (θ75 máx.) | 18 | ° | **ESTIMADO** | classe H135 (estimativa do modelo) | θ75 de -2° a 16° |
| Inércia de rolagem | 1.200 | kg m² | **ESTIMADO** | classe H135 (estimativa do modelo) | classe 3 t |
| Inércia de arfagem | 4.500 | kg m² | **ESTIMADO** | classe H135 (estimativa do modelo) | classe 3 t |
| Inércia de guinada | 3.800 | kg m² | **ESTIMADO** | classe H135 (estimativa do modelo) | classe 3 t |
| Meia bitola do esqui | 1,1 | m | **ESTIMADO** | classe H135 (estimativa do modelo) | geometria da classe |
| Meio comprimento do esqui | 1,3 | m | **ESTIMADO** | classe H135 (estimativa do modelo) | geometria da classe |
| Altura do CG sobre o esqui | 1,3 | m | **ESTIMADO** | classe H135 (estimativa do modelo) | geometria da classe |
<!-- PARAMS:END -->

Além da tabela:
- a **área de arrasto f = 1,37 m²** é CALIBRADA (item 4);
- os **coeficientes da curva empírica de VRS** vêm da literatura (Johnson 1980 / Leishman 2006), a conferir na fonte primária;
- os **ganhos do SAS e do governador** são de projeto do controlador, não da aeronave.
