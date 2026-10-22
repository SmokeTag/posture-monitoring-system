# Guia — `analysis/live_posture.py` (visão ao vivo do sistema)

Janela ao vivo da unidade da cadeira (`firmware/chair`, o "cérebro"): o IMU do
corpo e os 8 FSRs da cadeira, mais o mapeamento canal ↔ posição dos FSRs.

## Rodar

Da raiz do repositório, com a cadeira no USB (e o corpo ligado, se quiser o IMU):

```bash
.venv/bin/python analysis/live_posture.py
.venv/bin/python analysis/live_posture.py --log data/<nome>.csv     # também grava o stream
.venv/bin/python analysis/live_posture.py --port /dev/ttyACM0         # outra porta
```

Setup do venv: `analysis/README.md`. Feche qualquer `arduino-cli monitor` antes
(a porta serial é exclusiva).

## O que aparece

- **Esquerda — IMU:** desvio da postura calibrada `(dpitch, droll)` com rastro,
  e círculos de alerta (10°) e de liberação (7°). Caixa de status: desvio,
  decisão da cadeira (`chair`), o que o corpo relata ter aplicado (`body`),
  motor e enlace:
  - `body  chair, age 2` — a cadeira decide; o corpo segura a decisão tomada
    2 amostras antes desta (o normal: decidida em N, aplicada com o ACK de
    N+1, relatada em N+2). `age 254+` = muito velha; `?` = desconhecida.
  - `✓ applied` / `≠ applied` — confere o que o corpo aplicou (referência e
    alerta) com a linha da cadeira que tomou essa decisão. `unconfirmed`
    quando não dá para conferir (sem base, idade desconhecida/saturada,
    decisão fora do buffer, outra sessão do corpo) — não é divergência.
  - `body  FALLBACK (alone)` — sem ACKs válidos há 1 s, o corpo decide
    sozinho com a referência que tem (`ref = chair's` / `ref ≠ chair's`);
    o alerta dele pode legitimamente diferir do da cadeira.
  - `motor ● ON alert` / `● ON test` — saída do motor comandada (não medida),
    por alerta ou por teste de motor (`m`).
- **Direita — cadeira vista de cima, encosto rebatido para trás** (a esquerda de
  quem senta fica à esquerda da tela): 4 posições no assento + 4 no encosto.
  - Cor = carga no FSR (mais escuro = mais força; **cinza = solto**), com a
    resistência escrita no círculo.
  - Por placa: `+` = centro de pressão, divisão E/D e frente/trás (assento) ou
    baixo/cima (encosto), e % da carga total da cadeira.
- **Embaixo — um gráfico por canal (C8–C15)**, últimos 10 s, mesma escala para
  todos; o título diz a posição mapeada. Continuam funcionando com o corpo
  desligado.

## Saúde da IMU

No protocolo v3, o painel mostra `IMU OK`, `initializing`, `I2C error`,
`no fresh data` ou `invalid gravity vector`. São estados da amostra atual:
rádio conectado não implica amostra válida. Com dados inválidos, o ponto e o
rastro de postura somem; os FSRs continuam funcionando. A referência de
calibração é preservada, mas uma captura incompleta é abortada e alertas
posturais são interrompidos. O teste manual do motor permanece separado.

`IMU guard (inhibited)` indica que o corpo suprimiu uma decisão de alerta
anterior à falha; isso não é falha de aplicação. Logs v1/v2 continuam legíveis,
mas a saúde aparece como `IMU ? (old firmware)`. Um vetor inválido também pode
vir de movimento/queda livre; não é diagnóstico de sensor quebrado.

## Teclas (com a janela em foco)

O CSV da cadeira registra `c8..c15` sempre em ADC bruto de 12 bits (0–4095,
FSR aberto ≈0), com `# unit=raw` no cabeçalho. O painel converte os valores
para resistência/condutância no computador; também lê o formato antigo em
ohms. O cabeçalho sai automaticamente na conexão/reconexão USB; o painel
abre a porta com DTR baixo e o ativa após a abertura, sem enviar comandos.

| tecla | ação |
|---|---|
| `c` | calibrar a postura ereta (manda `c` para a cadeira; o botão físico faz o mesmo) |
| `r` | limpar a calibração |
| `m` | mapear os 8 FSRs |
| `Esc` | cancelar o mapeamento |
| `0`–`9` | tag da postura (`0` = transição); vai como coluna `tag` no `--log` ([protocolo](protocolo-captura.md)) |
| clique num círculo | remapear só aquela posição |

## Mapear os FSRs

Diz ao app qual canal do mux (C8–C15) está em cada posição da cadeira.

1. Aperte **`m`**. As posições são pedidas em ordem: assento trás E, trás D,
   frente E, frente D; encosto baixo E, baixo D, cima E, cima D.
   **E/D = de quem está sentado** (de frente para a cadeira, fica invertido).
2. Para cada uma, siga o aviso no topo do painel:
   **tire as mãos** (o app mede a linha de base) → **aperte e segure o sensor
   pedido** (~0,5 s) → o círculo fica verde → **solte**.
3. Errou ou trocou um? **Clique no círculo** e aperte o sensor certo. Enquanto
   uma posição espera, também dá para **clicar no gráfico do canal** lá embaixo.
   Se o canal já estava em outra posição, os dois trocam de lugar.

Detecção: o canal precisa subir ≥ 50 µS de condutância (resistência abaixo de
~20 kΩ a partir de solto), ficar 2× acima do segundo colocado e se manter
assim por 0,4 s. Sensores já carregados (alguém sentado) não atrapalham — vale
a subida em relação à linha de base.

O mapa fica em **`analysis/chair_map.json`** e é carregado ao abrir o app. Com
`--log`, a captura ganha uma linha `# fsr map c8=… c15=…`. Depois de mapear a
cadeira real, commite o `chair_map.json`: ele registra a fiação.

## Bateria do corpo

A linha `batt` mostra a tensão da bateria do corpo (coluna `vbat`, protocolo
v4), lida a cada 2 s; `LOW` abaixo de 3,4 V. Abaixo de 3,3 V o corpo desliga
sozinho (System OFF) e só volta com reset ou ligando o USB. No USB, o valor é
a saída do carregador.

## Problemas comuns

| sintoma | causa provável |
|---|---|
| "waiting for …" | cadeira desconectada ou porta ocupada (feche o monitor serial) |
| "no FSR data yet (old chair firmware?)" | firmware da cadeira sem as colunas `c8..c15` — regrave `firmware/chair` |
| todos os círculos cinza | nada pressionado (FSR solto lê ≈0 no ADC; em logs antigos em ohms, `-1` = aberto) |
| o mapeamento não reconhece o toque | aperte com mais força (precisa < ~20 kΩ); ou clique no gráfico do canal |
| "BODY SILENT" | unidade corporal desligada ou fora de alcance — os FSRs continuam |
| "BODY SILENT … low-battery cutoff?" | a última leitura da bateria do corpo era < 3,4 V: provavelmente desligou por bateria baixa (< 3,3 V); recarregue pelo USB |
| "BODY PROTOCOL MISMATCH (ver=N)" | versões de protocolo diferentes — grave as **duas** placas com a mesma versão (v4) |
| "BODY PACKETS REJECTED (bad=N)" | payloads malformados/incoerentes; não é o contador de falhas da IMU |
| "IMU … / posture sample unavailable" | rádio pode estar ativo, mas não há amostra de orientação válida |
