# Fontes do template "Helicóptero UTI"

Todas as URLs foram abertas e conferidas em 04/10/2026. Valores sem fonte pública aparecem como **ESTIMADO** em `src/physics/helicopter/params.py` e em `docs/helicoptero-uti.md`.

## Aeronave de referência (classe H135)

| Fonte | O que foi usado | URL |
|---|---|---|
| Airbus Helicopters — H135 technical information | MTOW 2.980 kg; teto de pairado OGE 7.200 ft ISA (massa não informada); diâmetro 10,40 m (divergente do TCDS); "Performance data @ MTOW — Max speed: 140 kts" (2ª validação). A página rotula 136 kt como "Maximum speed (VNE)", o que conflita com o VNE de 155 KIAS do TCDS; por isso o 136 kt foi tomado do folheto, onde aparece como cruzeiro rápido (reconferido em 04/10/2026) | https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information |
| Folheto Airbus H135 (fev. 2022) | combustível de 560 kg no tanque padrão; "Fast Cruise Speed: 136kts" (sem condições declaradas), usado na calibração da área de arrasto | https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html |
| EASA TCDS R.009, Issue 20 (dez. 2025) | rotor de Ø 10,20 m e 4 pás; rotor de cauda carenado de Ø 1,00 m com 10 pás; limites de NR; limites de torque AEO/OEI (EC135 P2/P3); base de certificação "JAR-27 ... CS-27 Amdt. 2" e "For CAT A Certification: CS-27 Amdt. 2, Appendix C requirements" (EC135 P3, T3 e variantes H); VNE 155 KIAS ao nível do mar | https://www.easa.europa.eu/en/downloads/7943/en |
| EASA TCDS R.009, Issue 05 (jan. 2014) | limites de torque (2 × 75 %, 2 × 69 %, 128 %, 125 %, 86 %) | https://www.easa.europa.eu/sites/default/files/dfu/certification-type-certificates-docs-rotorcraft-EASA-TCDS-R.009_Airbus_Helicopters_Deutschland_EC135-05-07012014.pdf |
| TCDS IM.E.017, Issue 07 (out. 2015) — PW206B3 | potências por regime: decolagem 336, máx. contínua 324, OEI 30 s 547 e OEI 2 min 534 kW | https://www.caa.co.uk/Documents/Download/3939/984aa08e-5352-4da9-824b-309ec7cec37d/2962 |
| Kampa et al., ERF 1997 (projeto do EC135) | corda equivalente de 0,288 m; velocidade de ponta de 211 m/s; corda (0,05 m) e velocidade de ponta (188 m/s) do rotor de cauda; deriva "small fin (0.9 m²)" da configuração básica VFR; "Vcruise 141 kts" e "Vne 155 kts" (EC135 de 1997, massa não informada) | https://dspace-erf.nlr.nl/bitstreams/8af39742-be07-4807-b1f4-d15ac616f12b/download |
| Doleschel & Emmerling, ERF 2007 | 395 rpm a 100 % NR; 100 % de torque = 665 N·m a 5.898 rpm | https://dspace-erf.nlr.nl/bitstreams/02b3ce46-b124-4c27-892e-b3cd0d2021cf/download |

## Passo 3: interior UTI, massa e CG, consumo e raio de ação

| Fonte | O que foi usado | URL |
|---|---|---|
| Airbus Helicopters — H135 technical information | carga útil 1.418 kg (→ massa vazia 1.562 kg, DERIVADO); "Max range (with std. fuel tank) 633 km / 342 NM" e "Max endurance (with std fuel tank) 3h 36 min", condições não informadas, usados na calibração do consumo (conferido em 04/10/2026) | https://www.airbus.com/en/products-services/helicopters/civil-helicopters/h135/h135-technical-information |
| Folheto Airbus H135 (fev. 2022) | combustível "Standard 560kg, with auxiliary 730kg" | https://pdf.aeroexpo.online/pdf/airbus-helicopters/h135/173989-29487.html |
| EASA TCDS R.009, seção 18 (EC135 T3H) | envelope de CG: dianteiro 4.180 mm a 1.840 kg e 4.237,5 mm a 3.175 kg; traseiro 4.570 mm a 1.500 kg e 4.349 mm a 3.175 kg; lateral ±100 mm; plano de referência 2.160 mm à frente do ponto de nivelamento | https://www.easa.europa.eu/en/downloads/7943/en |
| EASA AMC2 CAT.POL.MAB.100(d) (Regulatory Library da UK CAA, versão retida do Reg. UE 965/2012) | "85 kg for flight crew/technical crew members" | https://regulatorylibrary.caa.co.uk/965-2012/Content/Document%20Structure/04%20CAT/3%20AMC/AMC2%20CAT%20POL%20MAB%20100%20d%20Mass.htm |
| Hamilton Medical — HAMILTON-T1 | ventilador de transporte, 6,5 kg (unidade de ventilação) | https://www.hamilton-medical.com/en_US/Products/Mechanical-ventilators/HAMILTON-T1.html |
| ZOLL — X Series | monitor/desfibrilador, "less than 5.5 kilograms" | https://www.zoll.com/en-gb/products/defibrillators/x-series-for-hospital |
| Portaria GM/MS nº 2.048/2002 (vigente como ato próprio; ver abaixo) | lista de equipamentos da aeronave de transporte médico de asa rotativa | https://bvsms.saude.gov.br/bvs/saudelegis/gm/2002/prt2048_05_11_2002.html |
| ANAC — RBAC nº 91, Emenda 08 (02/10/2026), 91.151(b) | **reserva padrão**: helicóptero VFR, combustível até o primeiro pouso previsto + 20 min no consumo normal de cruzeiro (texto citado em docs/helicoptero-uti.md) | https://pergamum.anac.gov.br/pergamum/vinculos/RBAC91EMD08.pdf |
| ANAC — RBAC nº 135, Emenda 15 (vigência 09/01/2026) | a seção "135.209 Autonomia para voo VFR" consta do índice da Subparte D, mas o texto não pôde ser lido (o conteúdo recebido termina na 135.128; o download pela linha de comando foi bloqueado pela rede do ambiente): **lacuna** | https://pergamum.anac.gov.br/arquivos/RBAC135EMD15.pdf |
| RBAC nº 135, Emenda 13, cópia secundária no Scribd (informada pelo autor do projeto; não lida por mim; URL não registrada) | **indício, não fonte confirmada**: 135.209(b) exigiria, para helicóptero VFR, combustível até o destino + 20 min no consumo normal de cruzeiro. Não usado como fonte de parâmetro; aguarda o PDF oficial da Emenda 15 | — |
| 14 CFR 135.209(b) (texto via LII/Cornell; o eCFR recusou a conexão) | comparação: reserva VFR de helicóptero de 20 min no consumo normal de cruzeiro | https://www.law.cornell.edu/cfr/text/14/135.209 |
| Reg. (UE) 965/2012, Anexo V, SPA.HEMS.125(c) e GM1 SPA.HEMS.125(c)(3) (EASA Easy Access Rules, versão desde 25/05/2024) | classe de desempenho em hospital e em local de operação HEMS; citações curtas em docs/helicoptero-uti.md | https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-air-operations?page=50 |
| Reg. (UE) 965/2012, Anexo I, definições (85) e (86), texto original de 2012 | definições das classes de desempenho 1 e 2 | https://www.legislation.gov.uk/eur/2012/965/annexes/adopted/data.xht?view=snippet&wrap=true |
| Reg. (UE) 965/2012, Anexo I, definição (63) "HEMS operating site" (EASA) | definição do local de operação HEMS | https://www.easa.europa.eu/en/downloads/137085/en |
| ANAC — RBAC nº 90, Emenda 02 | não menciona classe de desempenho; 90.301 trata de pouso e decolagem em local não cadastrado | https://pergamum.anac.gov.br/pergamum/vinculos/RBAC90EMD02.pdf |
| Ministério da Saúde — SAMU 192, legislação | Portaria nº 2.048/2002 listada como ato próprio, ao lado das Portarias de Consolidação nº 3 e nº 6/2017 (conferido em 04/10/2026) | https://www.gov.br/saude/pt-br/composicao/saes/samu-192/legislacao |
| Ministério da Saúde — matriz da Portaria de Consolidação nº 3/2017 | não inclui a Portaria 2.048/2002 entre as normas consolidadas | https://bvsms.saude.gov.br/bvs/saudelegis/gm/2017/MatrizesConsolidacao/Matriz-3-Redes.html |

## Teoria e critérios

| Fonte | Uso | URL |
|---|---|---|
| W. Johnson, *Model for Vortex Ring State Influence on Rotorcraft Flight Dynamics*, NASA/TP-2005-213477 | critério de VRS (−0,5 a −1,5·v_h; V_x < v_h) | https://ntrs.nasa.gov/api/citations/20060024029/downloads/20060024029.pdf |
| Cheeseman & Bennett, ARC R&M 3021 (1955) | efeito solo; fórmula de pairado conferida no PDF | https://reports.aerade.cranfield.ac.uk/bitstream/handle/1826.2/3590/arc-rm-3021.pdf?sequence=1&isAllowed=y |
| J. G. Leishman, *Principles of Helicopter Aerodynamics*, 2ª ed., Cambridge, 2006 | teoria do momento e de elemento de pá, κ, fator 4,65μ², curva empírica de descida, ventilador carenado | livro, sem URL |
| G. D. Padfield, *Helicopter Flight Dynamics*, 2ª ed., Blackwell, 2007 | constante de tempo do *flapping* 16/(γΩ); dados do Bo105 como ordem de grandeza (inércia, número de Lock) | livro, sem URL |
| ICAO Doc 7488 / ISO 2533 | atmosfera padrão | norma |

## Regulação

Os itens do 14 CFR e do CAT.POL.H.205 abaixo foram usados como **critérios do modelo** na demonstração Categoria A e no diagrama H-V (Passo 2). Isso não é verificação de conformidade. Os demais documentos são só referências.

| Documento | URL |
|---|---|
| 14 CFR 29.67(a)(1) — subida OEI ≥ 100 ft/min na VTOSS, potência OEI 2 min, fora do efeito solo (texto vigente no eCFR, conferido em 04/10/2026) | https://www.ecfr.gov/current/title-14/section-29.67 |
| 14 CFR Part 27, Apêndice C — critérios Categoria A para helicópteros pequenos multimotores: exige, entre outros, os §§ 29.53, 29.59, 29.60, 29.65(a), 29.67(a), 29.77, 29.79 e 29.87(a) (eCFR, conferido em 04/10/2026). O TCDS da EASA cita o equivalente europeu, CS-27 Amdt. 2 Apêndice C, cujo texto não abri | https://www.ecfr.gov/current/title-14/chapter-I/subchapter-C/part-27/appendix-Appendix%20C%20to%20Part%2027 |
| 14 CFR 29.59(a)(1) e (c) — a trajetória de decolagem Cat A deve ficar fora do envelope H-V do § 29.87; na decolagem continuada, não descer abaixo de 15 ft acima da superfície de decolagem quando o TDP está acima de 15 ft (eCFR, conferido em 04/10/2026) | https://www.ecfr.gov/current/title-14/section-29.59 |
| 14 CFR 29.60(a)(2) e (a)(3) — heliponto elevado, Cat A: ao buscar a VTOSS a aeronave pode descer abaixo do nível da superfície de decolagem se, ao cruzar a borda do heliponto elevado, todas as partes passarem a pelo menos 15 ft de todos os obstáculos; a magnitude da descida deve ser determinada (eCFR, conferido em 04/10/2026) | https://www.ecfr.gov/current/title-14/section-29.60 |
| 14 CFR 29.87(a) — envelope altura-velocidade: combinações de altura e velocidade (inclusive pairado) em que não se consegue pouso seguro após a falha do motor crítico (eCFR, conferido em 04/10/2026) | https://www.ecfr.gov/current/title-14/section-29.87 |
| 14 CFR 29.725 — ensaio de queda do trem: altura de queda de pelo menos 8 in (eCFR, conferido em 04/10/2026); usada para derivar o limite de 2,0 m/s de toque do critério de pouso seguro do H-V | https://www.ecfr.gov/current/title-14/section-29.725 |
| Regulamento (UE) 965/2012, Anexo IV, CAT.POL.H.205(b)(4) — na decolagem continuada (classe de performance 1), margem vertical de pelo menos 10,7 m (35 ft) sobre os obstáculos até o fim da TODRH. Texto conferido na Regulatory Library da UK CAA (versão retida do mesmo regulamento), em 04/10/2026; o texto consolidado da UE no EUR-Lex não pôde ser aberto pela ferramenta | https://regulatorylibrary.caa.co.uk/965-2012/Content/Document%20Structure/04%20CAT/2%20Regs/08610_CAT.POL.H.205.htm |
| Mesma regra, texto oficial da UE (EUR-Lex, não conferido por mim) | https://eur-lex.europa.eu/eli/reg/2012/965/oj |
| ANAC — índice dos RBAC | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac |
| RBAC 27 (helicópteros categoria normal) | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac/rbac-27 |
| RBAC 29 (helicópteros categoria transporte) | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac/rbac-29 |
| RBAC 135 (operações complementares e por demanda) | https://www.anac.gov.br/assuntos/legislacao/legislacao-1/rbha-e-rbac/rbac/rbac-135 |
| Portaria GM/MS nº 2.048/2002 (inclui a aeronave de transporte médico; item 3.5.1 trata dos equipamentos de asa rotativa) | https://bvsms.saude.gov.br/bvs/saudelegis/gm/2002/prt2048_05_11_2002.html |

**Observação:** a Portaria 2.048/2002 continua vigente como ato próprio (ver as duas fontes do Ministério da Saúde na seção do Passo 3); não conferi as matrizes das Portarias de Consolidação além da nº 3.
