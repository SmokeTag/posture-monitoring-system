# Protocolo — captura do sistema completo (IMU + 8 FSRs)

Uma sessão gravada com todas as posturas rotuladas, para ajustar e validar a
classificação (TASKS.md, item 4). ~20–25 min por sessão. Baseado no protocolo
antigo do IMU ([`archive/protocolo-captura-imu.md`](archive/protocolo-captura-imu.md)).

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
- Se as pernas cruzadas geram alerta ainda está em aberto.

## Antes de começar

- Cadeira com os FSRs fixados e mapeados (`m` no painel; o mapa vai para o log
  como `# fsr map`). Corpo no colar/invólucro, na posição de uso.
- Anote: **data · sujeito · versão do colar/invólucro · cadeira** e qualquer
  mudança de montagem. Arquivo:
  `data/YYYY-MM-DD_<sujeito>_run<N>.csv`.

```bash
.venv/bin/python analysis/live_posture.py --log data/<arquivo>.csv
```

Confira no painel: `IMU OK`, enlace sem perdas, os 8 FSRs respondendo, e
`tag 0 · transition` sem `(not logging)`.

## Sequência

Cada postura: **~15 s parado**, `0` em toda movimentação. Mantenha durações
parecidas (os FSR402 têm *creep*: a leitura sobe devagar sob carga constante).

1. **Calibração.** Sente ereto, relaxado, como você realmente senta. Espere
   assentar, aperte o botão da cadeira (ou `c`) e aguarde o LED verde. Marque
   a tag da postura ereta por ~15 s.
2. **Série marcada.** Todas as posturas `1`–`9`, bem caracterizadas, **3 vezes,
   em ordem embaralhada** a cada repetição (para a deriva não se confundir com
   uma postura). Volte ao ereto entre elas quando fizer sentido.
3. **Série sutil.** Igual, mas cada postura do jeito que você *naturalmente*
   escorrega para ela. É aqui que aparece a margem real (no protocolo antigo,
   curvado ↔ cabeça baixa).
4. **Recolocação (3–4×).** Levante, tire o colar, recoloque, sente, recalibre e
   faça uma série curta (ereto + 2–3 posturas). Mede a deriva da montagem e da
   forma de sentar.
5. **Bloco livre (opcional, 3–5 min).** Uso normal (digitar, ler, celular),
   rotulando o que for reconhecível e `0` no resto. Inclui combinações
   tronco + pernas.

## Depois

- Feche a janela (o log já está gravado).
- Registre a run em [`../data/README.md`](../data/README.md) (série marcada/sutil,
  quantas recolocações, imprevistos). Run com problema (sensor solto, enlace
  caindo, tags trocadas) não é commitada.

## Checklist de qualidade

- Toda movimentação coberta por `0`; nenhuma postura com menos de ~10 s.
- `IMU OK` e enlace sem perdas durante a sessão; os 8 FSRs mapeados.
- Calibração refeita a cada recolocação.
- Cabeçalho do log com `# tag map` e `# fsr map`.
