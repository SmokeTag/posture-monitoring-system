# Protocolo — captura do sistema completo (IMU + 8 FSRs)

Uma sessão gravada com todas as posturas rotuladas, para ajustar e validar a
classificação (TASKS.md, item 4). Duas versões: a **curta** (~12 min, a usada
nas capturas de 21–22/10) e a **completa** (opcional, para a deriva da
recolocação). Baseado no protocolo antigo do IMU
([`archive/protocolo-captura-imu.md`](archive/protocolo-captura-imu.md)).

## Tags

Rotula-se pressionando `0`–`9` na janela do `analysis/live_posture.py`. A tag
fica valendo até a próxima tecla e vai como última coluna (`tag`) de cada linha
do log.

- **`0` = transição.** Pressione *antes* de começar a se mover e só troque para a
  postura depois de parado nela. É o estado inicial do log: tudo o que não foi
  rotulado de propósito é descartado na análise.
- **`1`–`9` = posturas**, fixas a partir da run 1. Os nomes ficam em
  `analysis/tags.json`, que o painel mostra e grava no cabeçalho do log (linha
  `# tag map`).

| tag | postura | como fazer |
|----:|---------|------------|
| `0` | transição | em movimento (descartada) |
| `1` | `ereto` | referência, a mesma da calibração |
| `2` | `curvado_frente` | tronco curvado, costas fora do encosto |
| `3` | `cabeca_frente` | tronco no encosto, cabeça/pescoço projetados à frente |
| `4` | `escorregado` | quadril escorregado para frente, só as escápulas no encosto |
| `5` | `inclinado_esq` | tronco inclinado para a esquerda (apoiado no braço) |
| `6` | `inclinado_dir` | idem, para a direita |
| `7` | `reclinado` | encosto inteiro, quadril atrás (postura aceitável) |
| `8` | `perna_e_sobre_d` | tronco ereto, perna esquerda cruzada sobre a direita |
| `9` | `perna_d_sobre_e` | tronco ereto, perna direita cruzada sobre a esquerda |

- **Pernas** (`8`/`9`): o IMU quase não as vê, quem vê é o assento.
  Combinações (curvado + perna cruzada) ficam para o bloco livre, assim como
  posturas fora da tabela (sentado na ponta, tornozelo sobre o joelho).
- Pernas cruzadas não geram alerta (`docs/decisao-fusao-imu-fsr.md`).

## Antes de começar

- **Mapa dos FSRs:** sentado, aperte o canto traseiro **esquerdo** do assento →
  deve subir o **C10**. O mapa (`analysis/chair_map.json`, gravado no log como
  `# fsr map`) tem que bater com o `FSR_SPOT_CH` do `firmware/chair/chair.ino`;
  remapear com `m` exige atualizar essa tabela também.
- **Motor:** para coletar dados, grave o corpo com o motor mudo
  (`-DMOTOR_MUTED=1`, `firmware/README.md`), porque a vibração mexe o IMU. A
  coluna `bmotor` continua indicando quando ele teria vibrado. Para validar o
  alerta, use o firmware normal. Anote qual foi usado.
- Corpo no colar/invólucro, na posição de uso. Anote: **data · sujeito ·
  versão do colar/invólucro · cadeira · motor mudo ou ativo** e qualquer
  mudança de montagem.
- **Arquivo novo a cada sessão** (`--log` recusa um arquivo que já existe):
  `data/YYYY-MM-DD_<Nome>_run<N>.csv`, com `run2`, `run3`... para a mesma pessoa.

```bash
.venv/bin/python analysis/live_posture.py --log data/<arquivo>.csv
```

Confira no painel: `IMU OK`, enlace sem perdas, os 8 FSRs respondendo e o
badge `TAG 0 · transition` (borda vermelha = não está gravando).

## Regras de cada postura

- **`0` antes de se mover**; a tag da postura só depois de parado nela.
- Segure cada postura **~15 s**, com durações parecidas (os FSR402 têm
  *creep*: a leitura sobe devagar sob carga constante).
- **~10 s de ereto (`1`) entre posturas:** `0` → endireita → `1` → `0` →
  próxima.
- **Depois do `4` (escorregado), puxe o quadril para trás** antes da próxima
  (senão a próxima sai misturada com escorregado).
- **Calibre sentado até o fundo da cadeira.** Na ponta do assento a calibração
  é rejeitada (LED vermelho rápido por 2 s, `# calibrate rejected` no log):
  sente direito e calibre de novo.

## Protocolo curto (~12 min, padrão)

1. **Calibração.** Ereto e relaxado, como a pessoa realmente senta → botão da
   cadeira (ou `c`) → espere o LED verde → `1` por 15 s.
2. **Passada A**, depois **passada B**: as 8 posturas na ordem da folha.
3. **Sutil:** 2, 4, 5 e 6 na ordem da folha, cada uma *do jeito que a pessoa
   naturalmente escorrega para ela* (leve, sem exagero).
4. **Recolocação:** levante, tire o colar, recoloque, sente, **recalibre** e
   faça a coluna "Recolocação".
5. `0`, feche a janela.

Uma linha da folha por pessoa, sem repetir (P1 e P2 já foram usadas; a
próxima é P3). A ordem embaralhada evita que a deriva e o *creep* se
confundam com uma postura.

| Folha | Passada A | Passada B | Sutil | Recolocação |
|---|---|---|---|---|
| P1 | 4 7 3 9 5 2 6 8 | 7 4 6 3 2 8 5 9 | 5 4 6 2 | 1 4 2 |
| P2 | 5 6 2 9 8 7 4 3 | 5 9 4 6 8 2 7 3 | 5 2 6 4 | 1 7 4 |
| P3 | 5 4 8 7 3 2 9 6 | 7 5 4 9 6 2 8 3 | 6 5 4 2 | 1 2 7 |
| P4 | 7 3 5 2 8 6 9 4 | 9 2 3 8 4 6 7 5 | 6 4 5 2 | 1 4 2 |
| P5 | 4 8 5 3 6 2 9 7 | 3 4 7 9 6 8 2 5 | 5 6 2 4 | 1 7 4 |
| P6 | 2 7 8 3 5 9 6 4 | 8 4 6 7 5 3 2 9 | 2 5 6 4 | 1 7 2 |
| P7 | 8 2 9 3 4 7 5 6 | 9 8 5 6 4 3 2 7 | 6 2 5 4 | 1 2 4 |
| P8 | 4 8 5 7 3 9 6 2 | 5 9 6 7 4 2 8 3 | 2 5 6 4 | 1 7 2 |

## Protocolo completo (opcional)

Para medir a deriva da recolocação (resultado do relatório) e a margem real
das posturas. Mesmas regras; ~25 min.

1. Calibração, como no curto.
2. **Série marcada:** as posturas `2`–`9`, **3 vezes, cada uma em ordem
   embaralhada** (dá para usar 3 passadas de linhas diferentes da folha).
3. **Série sutil:** todas as posturas ruins, do jeito natural. É aqui que
   aparece a margem real (no protocolo antigo, curvado ↔ cabeça baixa).
4. **Recolocação (3–4×):** levante, tire o colar, recoloque, sente,
   recalibre e faça ereto + 2–3 posturas.
5. **Bloco livre (3–5 min):** uso normal (digitar, ler, celular), rotulando o
   que for reconhecível e `0` no resto. Inclui combinações tronco + pernas.

## Depois

- Registre a run em [`../data/README.md`](../data/README.md): versão
  (curta/completa), linha da folha e o que foi feito dela, recalibrações,
  motor mudo/ativo, imprevistos (tecla errada, sensor solto, postura
  misturada). Run com problema (sensor solto, enlace caindo, tags trocadas)
  não é commitada.

## Checklist de qualidade

- Toda movimentação coberta por `0`; nenhuma postura com menos de ~10 s.
- `IMU OK` e enlace sem perdas durante a sessão; os 8 FSRs mapeados.
- Calibração refeita a cada recolocação.
- Cabeçalho do log com `# tag map` e `# fsr map`.
