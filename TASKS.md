# Tarefas

Roteiro de desenvolvimento em andamento + backlog. Cada item tem contexto
suficiente para ser retomado a frio. Ao concluir, **remova o item**; decisões
que surgirem vão para o `CLAUDE.md`.

## Roteiro do firmware

Etapas em ordem; cada uma é testável isoladamente antes da seguinte.
Toolchain/FQBN: cabeçalho de `firmware/xiao_imu_test/xiao_imu_test.ino`.

### [ ] 1. Leitura dos 8 FSRs na PCB da cadeira

Sketch `firmware/chair/` pronto e validado em C8 e C10–C15.
**Falta (bancada):**
- [ ] C9 está em curto com o 3V3 na PCB (lê fundo de escala sem FSR).
      Corrigir o curto e confirmar que pressionar o FSR do C9 altera só o C9.
- [ ] Botão de calibração: soldar entre **D9** e GND (firmware pronto:
      `INPUT_PULLUP` + debounce por software, sem RC). Testar: apertar →
      `# calibrate queued (button)` → `# calibrate sent` e o LED verde pisca
      (se não piscar, a polaridade `LED_ON` do LED está invertida).

### [ ] 3. Firmware do corpo: IMU + motor

Partir de `firmware/body/` (receptor ESB), trazer de `xiao_imu_test.ino`
o pitch/roll + calibração por desvio (disparada pelo comando que chega via
ESB) e adicionar o motor de vibração (driver transistor + diodo de roda
livre) com padrão de alerta. O `loop()` não pode bloquear (ver ESB no
`CLAUDE.md`). Lógica inicial só com IMU: desvio além do limiar por
X segundos → vibra.
**Pronto quando:** inclinar-se além do limiar por X s aciona o motor e
voltar à postura calibrada o desliga.

### [ ] 4. Fusão cadeira + IMU e classificação

Combinar a distribuição de pressão (assento/encosto, presença, assimetria
E/D) com o desvio do IMU; definir posturas-alvo, limiares e temporização
(histerese, sem alertas em transições). Log serial CSV para ajustar
limiares com dados reais.
**Pronto quando:** as posturas do protocolo (`docs/protocolo-captura-imu.md`)
são classificadas corretamente em sessões gravadas.

### [ ] 5. Integração, montagem e validação

FSRs fixados na cadeira (4 assento + 4 encosto), botão físico de calibração
na unidade da cadeira, unidade corporal na bateria (ver item da LiPo abaixo),
leitura da tensão da bateria. Validação final com sessões de uso para o
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

### Arquitetura do sistema

#### [ ] Decidir: app companheiro / dashboard / registro de dados, ou totalmente autônomo?

**Contexto:** hoje o sistema é autônomo — a unidade corporal (XIAO) funde os
dados da cadeira (via ESB) com o IMU e aciona o motor de vibração, sem nenhuma
interface externa.

**A decidir:**
- [ ] Standalone puro **ou** algum complemento: app de celular, dashboard web,
      ou só registro de dados (ex.: log em flash / serial para análise).
- [ ] Se houver app/celular: o elo com o telefone seria **BLE**, convivendo com o ESB cadeira→cérebro.
- [ ] Impacto em escopo/prazo do TCC, consumo de bateria e firmware.
- [ ] Registrar a decisão no `CLAUDE.md`.
