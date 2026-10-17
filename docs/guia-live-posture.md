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
  sobre os clusters de uma captura gravada; círculos de alerta (10°) e de
  liberação (7°). Caixa de status: desvio, estado do alerta, motor/LED, enlace.
- **Direita — cadeira vista de cima, encosto rebatido para trás** (a esquerda de
  quem senta fica à esquerda da tela): 4 posições no assento + 4 no encosto.
  - Cor = carga no FSR (mais escuro = mais força; **cinza = solto**), com a
    resistência escrita no círculo.
  - Por placa: `+` = centro de pressão, divisão E/D e frente/trás (assento) ou
    baixo/cima (encosto), e % da carga total da cadeira.
- **Embaixo — um gráfico por canal (C8–C15)**, últimos 10 s, mesma escala para
  todos; o título diz a posição mapeada. Continuam funcionando com o corpo
  desligado.

## Teclas (com a janela em foco)

| tecla | ação |
|---|---|
| `c` | calibrar a postura ereta (manda `c` para a cadeira; o botão físico faz o mesmo) |
| `r` | limpar a calibração |
| `m` | mapear os 8 FSRs |
| `Esc` | cancelar o mapeamento |
| clique num círculo | remapear só aquela posição |

## Mapear os FSRs

Diz ao app qual canal do mux (C8–C15) está em cada posição da cadeira.

1. Aperte **`m`**. As posições são pedidas em ordem: assento trás E, trás D,
   frente E, frente D; encosto baixo E, baixo D, cima E, cima D.
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

## Problemas comuns

| sintoma | causa provável |
|---|---|
| "waiting for …" | cadeira desconectada ou porta ocupada (feche o monitor serial) |
| "no FSR data yet (old chair firmware?)" | firmware da cadeira sem as colunas `c8..c15` — regrave `firmware/chair` |
| todos os círculos cinza | nada pressionado (FSR solto lê `-1` = aberto) |
| o mapeamento não reconhece o toque | aperte com mais força (precisa < ~20 kΩ); ou clique no gráfico do canal |
| "BODY SILENT" | unidade corporal desligada ou fora de alcance — os FSRs continuam |
