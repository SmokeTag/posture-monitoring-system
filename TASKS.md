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
Capturas seguem `docs/protocolo-captura.md` (tag `0` = transição; posturas
`1`–`9` em `analysis/tags.json`, incluindo pernas cruzadas, que não
alertam).
Regra de fusão implementada (`docs/decisao-fusao-imu-fsr.md`, `Fusion.h`);
falta validar com uma pessoa nova.
**Pronto quando:** as posturas desse protocolo são classificadas corretamente
em sessões gravadas.

### [ ] 5. Integração, montagem e validação

Botão físico de calibração na unidade da cadeira, testar a bateria da
cadeira (mesmo circuito do corpo), validar o corte por bateria baixa do
corpo na bancada (tensão no multímetro, corrente em System OFF, wake pelo
USB; ver `firmware/README.md`). Reajustar o limiar do IMU (10°/3 s pareceu
leniente) já no invólucro, medindo no `analysis/live_posture.py` onde cai uma
postura "levemente ruim". Validação final com sessões de uso para o
relatório.
**Pronto quando:** sistema completo funciona sem USB durante uma sessão
sentada.

## Backlog

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

### Firmware — alerta do motor

#### [ ] Padrão de vibração do alerta (menos insuportável)

**Contexto:** hoje o corpo pulsa o motor 700 ms a cada 1 s (duty 255)
(`PULSE_ON_MS`/`PULSE_PERIOD_MS` em `body.ino`) **sem parar** enquanto o
alerta durar. Ideia inicial (ainda a amadurecer), em ms, `+` ligado e `-`
desligado:
`+600 -1000 +200 -200 +200 -200 +200 -1000` ×3 → silêncio 3 min → repete
o bloco → silêncio 10 min.
**A pensar:** o que acontece depois dos 10 min (repete? fica quieto até
corrigir?); corrigir a postura reinicia o ciclo do zero ou só pausa
(evitar que endireitar e voltar a curvar "fure" o silêncio); se o padrão
roda só no corpo (vale também no fallback sem enlace) com a cadeira mandando
só o flag de alerta; LED vermelho continua espelhando o motor; padrão numa
tabela de constantes fácil de ajustar. Testar intensidade (`MOTOR_DUTY`) junto.
**Pronto quando:** padrão definido, implementado e testado no corpo
(com e sem enlace).

### Arquitetura do sistema

#### [ ] Decidir: app companheiro / dashboard / registro de dados, ou totalmente autônomo?

**Contexto:** hoje o sistema é autônomo — a unidade da cadeira (cérebro) funde
os FSRs com o IMU recebido do corpo (via ESB) e devolve o alerta; o corpo
aciona o motor de vibração, sem nenhuma interface externa.

**A decidir:**
- [ ] Standalone puro **ou** algum complemento: app de celular, dashboard web,
      ou só registro de dados (ex.: log em flash / serial para análise).
- [ ] Se houver app/celular: o elo com o telefone seria **BLE**, convivendo com o ESB corpo↔cadeira (cérebro).
- [ ] Impacto em escopo/prazo do TCC, consumo de bateria e firmware.
- [ ] Registrar a decisão no `CLAUDE.md`.

#### [ ] Melhorar a recepção do beacon BLE

No notebook MediaTek, testar scan passivo (`or_patterns`) com BlueZ experimental;
a recepção atual é best effort. Beacon e painel sem fio já implementados.
