# Procedimento de abortar v2 (com amortecimento): definição

> Simulador conceitual e educacional. Não é um simulador certificado (FSTD) nem substitui dados do fabricante.
> Este é um **modelo** de procedimento, montado a partir de descrições públicas. Não é o procedimento do manual de voo de nenhuma aeronave.

Este documento foi escrito e versionado **antes** de rodar o recálculo com o v2, como decidido no aval do Passo 4. Os parâmetros abaixo não foram ajustados a partir de resultados.

## Por que existe um v2

O procedimento de abortar original (**v1, sem amortecimento**) faz o seguinte depois do reconhecimento da falha:
- comanda descida vertical de 1,0 m/s acima de 3 m de altura dos esquis e de 0,3 m/s abaixo disso;
- mantém a proteção de queda de NR em 98,5 %: abaixo desse valor o coletivo é devolvido.

Com a detecção do toque corrigida, o v1 toca o deck a 3,3–4,0 m/s na massa máxima Categoria A antes aprovada. Com um só motor, a potência não basta para segurar a descida comandada. Além disso, a proteção de NR a 98,5 % tira coletivo justamente quando ele seria necessário para amortecer.

## Fontes públicas

1. **A. Garavello, S. Fuhr, R. Laporte**, *On the Establishment of Class 2 Helipad Takeoff and Landing Performance*, European Rotorcraft Forum (artigo no repositório da ERF): [dspace-erf.nlr.nl](https://dspace-erf.nlr.nl/server/api/core/bitstreams/955cf145-b292-4d91-b4fa-28efd8bb122c/content). A Seção 5.2 descreve o caso de falha de um motor (OEI) antes do ponto de rotação, durante a parte vertical da decolagem:
   - *"Collective lever is adjusted to 2.5 min power (Controlling NR 97% - 103.5%)"*;
   - *"Vertical Landing: cushion touchdown with collective"*.
2. **S. van 't Hoff, P. Hofmeister, L. Lu, G. D. Padfield, G. Quaranta, M. White**, *Rotorcraft Certification by Simulation and Analysis*, Springer, 2025, cap. "Case Study 2: CS 29/27 Category A Confined Area Rejected Take-Off": [link.springer.com](https://link.springer.com/chapter/10.1007/978-3-031-86398-1_13). Pelo resumo da editora:
   - o NR de 101 % deve ser recuperado depois da falha;
   - *"the collective should be adjusted to cushion the touchdown as required"*;
   - padrão de desempenho no toque: razão de descida abaixo de 400 ft/min (2,0 m/s, desejado) ou 500 ft/min (2,5 m/s, adequado).
   - Li só o resumo, não o capítulo completo.

## Definição do v2

| # | Etapa | Modelo | Base |
|---|---|---|---|
| 1 | Reconhecimento e reação | Igual ao v1: decisão no reconhecimento pelo SADPF, mais 1 s de reação | já aprovado (Passo 2) |
| 2 | Potência | Motor restante no regime OEI automático do modelo (30 s, depois 2 min) | fonte 1 (o artigo usa o regime de 2,5 min; o modelo tem 30 s / 2 min, do TCDS) |
| 3 | NR durante o abortar | A proteção de queda de NR passa de 98,5 % para **97 %**: o coletivo só é devolvido abaixo de 97 % | fonte 1 (faixa controlada de 97 a 103,5 %); 97 % é também o NR mínimo com motor do TCDS R.009 |
| 4 | Descida | Vertical, sobre o deck, comandada a 1,0 m/s (igual ao v1) | ESTIMADO (as fontes não dão a razão de descida) |
| 5 | Amortecimento | Abaixo de **5 m** de altura dos esquis (≈ um raio do rotor, onde o efeito solo começa a crescer), o comando de descida passa a 0,3 m/s. O v1 fazia isso só a 3 m | fontes 1 e 2 ("cushion touchdown with collective"); a altura de 5 m é ESTIMADA |
| 6 | Após o toque | Coletivo baixado a 1,5°/s (igual ao v1 corrigido) | ESTIMADO (correção de NR do aval do Passo 4) |

O v2 continua sendo um comando de razão de descida executado pelo mesmo SAS e pelo mesmo governador do modelo. O que muda:
- o coletivo pode usar a energia do rotor até 97 % de NR, sem ser devolvido a 98,5 %;
- o amortecimento começa mais alto, a 5 m em vez de 3 m.

## Critérios de sucesso (não mudam)

- **Toque ≤ 1,5 m/s**: margem operacional do modelo, ESTIMADA. É mais exigente que o "desejado" de 400 ft/min (2,0 m/s) da fonte 2.
- **Sem tombamento** (inclinação depois do toque < 15°, ESTIMADO) e **dentro do deck**, a 1 m da borda.
- **2,0 m/s é o limite estrutural de certificação**: queda livre de 8 in do ensaio de queda-limite do trem, [14 CFR 29.725(a)](https://www.ecfr.gov/current/title-14/section-29.725). Os dois valores coexistem:
  - 1,5 m/s é o critério de sucesso do abortar no modelo;
  - 2,0 m/s separa, na classificação do toque mostrada no HUD, o "pouso" do "pouso duro".
  - Um toque entre 1,5 e 2,0 m/s é um abortar **reprovado** pelo critério do modelo, embora a classificação o chame de "pouso".

## O que será recalculado com o v2

As mesmas grandezas do v1 (`tools/heli_reject_recalc.py v2`):
- massas Categoria A nos dois critérios e na faixa G 1,15–1,26;
- massa máxima no local de resgate (TDP 12 e 17 m);
- raio de ação limitado por massa;
- varreduras em torno do TDP;
- conferência das 130 previsões de ramos.
