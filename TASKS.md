# Tarefas

Roteiro de desenvolvimento em andamento + backlog. Cada item tem contexto
suficiente para ser retomado a frio. Ao concluir, **remova o item**; decisões
que surgirem vão para o `CLAUDE.md`.

## Roteiro do firmware

Etapas em ordem; cada uma é testável isoladamente antes da seguinte.
Toolchain/FQBN: `firmware/README.md`.

### [ ] 4. Fusão cadeira + IMU e classificação

Combinar a distribuição de pressão (assento/encosto, presença, assimetria
E/D) com o desvio do IMU; definir posturas-alvo, limiares e temporização
(histerese, sem alertas em transições). Log serial CSV para ajustar
limiares com dados reais.
Escrever o protocolo de captura completa (IMU + FSRs, com invólucro e colar),
reaproveitando o que funcionou no antigo (`docs/archive/protocolo-captura-imu.md`):
rotular as posturas com o **mapa de tags** (`0` ereto, `1` curvado para frente,
`2`/`3` inclinado E/D, `4` cabeça baixa, `5` reclinado) e a **tag `9` em toda
transição** (descartada na análise). Essa captura também alimenta o overlay de
referência do `analysis/live_posture.py`.
**Pronto quando:** as posturas desse protocolo são classificadas corretamente
em sessões gravadas.

### [ ] 5. Integração, montagem e validação

FSRs fixados na cadeira (4 assento + 4 encosto), botão físico de calibração
na unidade da cadeira, unidade corporal na bateria (ver item da LiPo abaixo),
leitura da tensão da bateria. Reajustar o limiar do IMU (10°/3 s pareceu
leniente) já no invólucro, medindo no `analysis/live_posture.py` onde cai uma
postura "levemente ruim". Validação final com sessões de uso para o
relatório.
**Pronto quando:** sistema completo funciona sem USB durante uma sessão
sentada.

## Backlog

### Hardware — unidade corporal

#### [ ] Ligação da bateria LiPo 1S (3 fios) ao XIAO nRF52840 Sense

**Contexto:** bateria **JYH LIP103450-1800 1S1P** (3,7 V, 1800 mAh, 6,6 Wh),
com **3 fios** e **conector de 3 vias**. Objetivo: alimentar o XIAO por ela e
recarregá-la pelo USB-C da própria placa, **sem cortar os fios da bateria**.

**Pesquisa feita (2026-09-29):** `docs/pesquisa-bateria-lipo.md`.
Resumo: ligar **só + e − nos pads BAT+/BAT−** e isolar o 3º fio (quase
certamente NTC 10 kΩ; o XIAO não tem onde ligá-lo) é seguro e funciona.
**Porém** o timer de segurança de ~10 h do carregador BQ25101, a 50/100 mA,
só enche ~28 %/~55 % dos 1800 mAh por conexão do USB.

**Falta (bancada/compra):**
- [ ] Medir o passo do conector (2,0 mm = JST-PH, 1,25 mm = PicoBlade,
      1,0 mm = JST-SH, 2,54 mm = JST-XH) e fotografar.
- [ ] Multímetro: vermelho↔preto = 3,0–4,2 V (confirma polaridade);
      3º fio↔preto ≈ 10 kΩ, caindo ao aquecer = NTC.
- [ ] Comprar o **par do conector com rabicho** (ex.: "JST PH 2.0 3 pinos com
      cabo"). Ao chegar, conferir de novo qual fio é + antes de soldar.
- [ ] **Decidir a estratégia de carga:** carregador do XIAO (P0.13 = LOW →
      100 mA no firmware + replugar o USB) **ou** carregador externo (TP4056,
      MCP73831, bq24074). Nunca usar os dois ao mesmo tempo.
- [ ] Testar: rodar só na bateria; com USB, confirmar carga (LED em P0.17).
- [ ] Firmware: ao ler a tensão da bateria, **nunca levar P0.14 a HIGH**
      (ver doc).

### Firmware — enlace ESB e energia

#### [ ] Retransmissões altas no enlace corpo→cadeira (+ economia de energia)

**Contexto:** desde que o cérebro foi para a cadeira (PR #2), o corpo vê
~10–35 retransmissões/s a 20 Hz (antes ~3/s), com 0 pacotes perdidos e no
máximo ~+20 ms de latência. Teste A/B: sem ACK payload ficam 3–10/s; com ele, a
`nrf_to_nrf` espera o ACK só ~0,8 ms (`ACK_TIMEOUT_1MBPS` + 200 µs) em vez de
~1,35 ms. O loop da cadeira não é a causa (~22 µs por volta; pico de ~2 ms =
2 ticks do FreeRTOS).
**Ideias:** ACK payload menor (refs em int16), 2 Mbps, ajustar o timeout na
lib. Junto: `radio.powerDown()` entre envios no corpo e tirar o
`TASKS_CONSTLAT` que a lib liga.
**Pronto quando:** retx/s perto do de antes e a corrente do corpo medida antes
e depois (número para o relatório).

#### [ ] Baixar a taxa do enlace para 5–10 Hz no produto final

**Contexto:** 20 Hz fica durante desenvolvimento/calibração (capturas mais
finas). Postura muda devagar e o alerta já espera 3 s, então 5–10 Hz basta.
**Cuidados:** taxa numa constante única no `PostureLink.h` (hoje
`SAMPLE_MS` no corpo e `SCAN_MS` na cadeira) e `LINK_TIMEOUT_MS`
proporcional a ela (~10 pacotes; a 5 Hz, ~2 s); amostrar o IMU mais rápido e
enviar a média (hoje é uma leitura crua por pacote); rever a nota de fase em
`chair.ino` (~l. 269); anotar a taxa das capturas no `data/README.md`. Só
economiza de fato junto com o sleep do item acima.
**Pronto quando:** enlace estável na taxa nova e corrente medida.

### Arquitetura do sistema

#### [ ] Decidir: app companheiro / dashboard / registro de dados, ou totalmente autônomo?

**Contexto:** hoje o sistema é autônomo — a unidade corporal (XIAO) funde os
dados da cadeira (via ESB) com o IMU e aciona o motor de vibração, sem nenhuma
interface externa.

**A decidir:**
- [ ] Standalone puro **ou** algum complemento: app de celular, dashboard web,
      ou só registro de dados (ex.: log em flash / serial para análise).
- [ ] Se houver app/celular: o elo com o telefone seria **BLE**, convivendo com o ESB corpo↔cadeira (cérebro).
- [ ] Impacto em escopo/prazo do TCC, consumo de bateria e firmware.
- [ ] Registrar a decisão no `CLAUDE.md`.

#### [ ] Considerar: monitor sem fio do estado via beacon BLE da cadeira

**Contexto:** ler o estado ao vivo sem o notebook no USB da cadeira e sem
comprar um 3º nRF. A `nrf_to_nrf` não tem BLE, mas o cérebro pode, a cada
~100–200 ms, pôr o RADIO em BLE 1 Mbit, enviar um `ADV_NONCONN_IND` nos canais
37/38/39 (~1–2 ms) e restaurar o ESB; o retransmit do corpo (~20 ms) cobre a
janela. Payload ~22 B: flags (cal/alert/led/link) 1 + dev 1 + pitch/roll int16
4 + 8 FSRs uint16 16. Leitura: script Python com `bleak` (só scan, sem
pareamento) imprimindo o mesmo CSV, ou o app nRF Connect no celular.
**Cuidados:** restaurar a config ESB por inteiro; nunca trocar no meio do
`radio.available()`; ~5–10 Hz efetivos com perdas (bom para monitorar/demo,
não substitui o USB nas capturas); medir o impacto nas retx/s. Alternativas
descartadas: BLE completo (SoftDevice conflita com a `nrf_to_nrf`); log na
flash QSPI (não é ao vivo, mas pode complementar).
**Pronto quando:** estado legível no notebook/celular com a cadeira fora do USB
e o enlace corpo↔cadeira sem perdas.
