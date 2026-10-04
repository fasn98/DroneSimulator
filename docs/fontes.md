# Fontes do template "Helicóptero UTI"

Todas as URLs foram abertas e conferidas em 04/10/2026. Valores sem fonte pública aparecem como **ESTIMADO** em `src/physics/helicopter/params.py` e em `docs/helicoptero-uti.md`.

## Aeronave de referência (classe H135)

| Fonte | O que foi usado | URL |
|---|---|---|
| Airbus Helicopters — H135 technical information | MTOW 2.980 kg; cruzeiro rápido 136 kt; teto de pairado OGE 7.200 ft ISA (massa não informada); diâmetro 10,40 m (divergente do TCDS) | https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information |
| Folheto Airbus H135 (fev. 2022) | combustível de 560 kg no tanque padrão | https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html |
| EASA TCDS R.009, Issue 20 (dez. 2025) | rotor de Ø 10,20 m e 4 pás; rotor de cauda carenado de Ø 1,00 m com 10 pás; limites de NR; limites de torque AEO/OEI (EC135 P2/P3) | https://www.easa.europa.eu/en/downloads/7943/en |
| EASA TCDS R.009, Issue 05 (jan. 2014) | limites de torque (2 × 75 %, 2 × 69 %, 128 %, 125 %, 86 %) | https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf |
| TCDS IM.E.017, Issue 07 (out. 2015) — PW206B3 | potências por regime: decolagem 336, máx. contínua 324, OEI 30 s 547 e OEI 2 min 534 kW | https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962 |
| Kampa et al., ERF 1997 (projeto do EC135) | corda equivalente de 0,288 m; velocidade de ponta de 211 m/s; corda (0,05 m) e velocidade de ponta (188 m/s) do rotor de cauda | https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download |
| Doleschel & Emmerling, ERF 2007 | 395 rpm a 100 % NR; 100 % de torque = 665 N·m a 5.898 rpm | https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download |

## Teoria e critérios

| Fonte | Uso | URL |
|---|---|---|
| W. Johnson, *Model for Vortex Ring State Influence on Rotorcraft Flight Dynamics*, NASA/TP-2005-213477 | critério de VRS (−0,5 a −1,5·v_h; V_x < v_h) | https://ntrs.nasa.gov/api/citations/20060024029/downloads/20060024029.pdf |
| Cheeseman & Bennett, ARC R&M 3021 (1955) | efeito solo; fórmula de pairado conferida no PDF | https://reports.aerade.cranfield.ac.uk/bitstream/handle/1826.2/3590/arc-rm-3021.pdf?sequence=1&isAllowed=y |
| J. G. Leishman, *Principles of Helicopter Aerodynamics*, 2ª ed., Cambridge, 2006 | teoria do momento e de elemento de pá, κ, fator 4,65μ², curva empírica de descida, ventilador carenado | livro, sem URL |
| G. D. Padfield, *Helicopter Flight Dynamics*, 2ª ed., Blackwell, 2007 | constante de tempo do *flapping* 16/(γΩ); dados do Bo105 como ordem de grandeza (inércia, número de Lock) | livro, sem URL |
| ICAO Doc 7488 / ISO 2533 | atmosfera padrão | norma |

## Regulação

Os dois itens do 14 CFR abaixo foram usados como **critérios do modelo** na demonstração Categoria A (Passo 2). Isso não é verificação de conformidade. Os demais documentos são só referências.

| Documento | URL |
|---|---|
| 14 CFR 29.67(a)(1) — subida OEI ≥ 100 ft/min na VTOSS, potência OEI 2 min, fora do efeito solo (texto vigente no eCFR, conferido em 04/10/2026) | https://www.ecfr.gov/current/title-14/section-29.67 |
| 14 CFR 29.59(c) — na decolagem continuada, não descer abaixo de 15 ft acima da superfície de decolagem quando o TDP está acima de 15 ft (eCFR, conferido em 04/10/2026) | https://www.ecfr.gov/current/title-14/section-29.59 |
| ANAC — índice dos RBAC | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac |
| RBAC 27 (helicópteros categoria normal) | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac/rbac-27 |
| RBAC 29 (helicópteros categoria transporte) | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac/rbac-29 |
| RBAC 135 (operações complementares e por demanda) | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac/rbac-135 |
| Portaria GM/MS nº 2.048/2002 (inclui a aeronave de transporte médico; item 3.5.1 trata dos equipamentos de asa rotativa) | https://bvsms.saude.gov.br/bvs/saudelegis/gm/2002/prt2048_05_11_2002.html |

**Observação:** não verifiquei se a Portaria 2.048/2002 foi incorporada à Portaria de Consolidação nº 3/2017. Isso precisa ser conferido antes de citá-la como vigente.
