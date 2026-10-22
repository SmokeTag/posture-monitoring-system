# Decisão de projeto: regra de alerta com fusão IMU + FSR

> **Status:** IMPLEMENTADA, falta validar com uma pessoa nova. Regra
> desenhada e avaliada em Python (`analysis/fusion.py`) e portada 1:1 para a
> cadeira (`firmware/libraries/PostureLink/Fusion.h`; `test_fusion` confere as
> duas em trechos reais dos logs). A queda de enlace no corpo continua só-IMU
> (o corpo não tem FSRs). O mapa posição ↔ canal dos FSRs está fixo no
> firmware (`FSR_SPOT_CH` em `chair.ino`, cópia de `analysis/chair_map.json`):
> remapear a cadeira exige atualizar essa tabela.

## Problema

A regra atual (`PostureLink.h`: desvio `dev = √(dpitch² + droll²)` > 10° por
3 s, limpa < 7°) só vê o IMU. Nas 9 capturas de 2026-10-21 (8 sujeitos,
`data/README.md`) ela: alerta em `reclinado` (postura aceitável) em 9/12
segmentos; perde `escorregado` quando o tronco quase não inclina (Rafael,
Thiago-Fita: droll +4–6°), embora os FSRs do assento traseiro zerem; e alerta
com a cadeira vazia (pessoa em pé com o colar). Com a montagem atual no colar,
**droll é o eixo sagital** (+ = para trás) e **dpitch o lateral**
(+ = direita) — conferido nos dados (tags 2, 4, 5, 6).

## Regra proposta

Entradas por amostra do corpo (20 Hz): `dpitch`, `droll` (graus, desde a
referência calibrada), os 8 FSRs crus (ADC 12 bits) por posição e a
**linha de base FSR** da calibração.

**Linha de base:** média de cada FSR nas **mesmas 20 amostras** (1 s) que
formam a referência do IMU (`CAL_SAMPLES` em `chair.ino`). Dela saem:

```
base_rear = b[seat_rear_left] + b[seat_rear_right]
bL = b[seat_front_left] + b[seat_rear_left];  bR = b[seat_front_right] + b[seat_rear_right]
rear_ok = base_rear >= BASE_MIN              # guarda: base ~0 desliga os termos do assento traseiro
lr_ok   = bL + bR  >= 2*BASE_MIN
base_lr = lr_ok ? (bL - bR)/(bL + bR) : 0
```

**Decisão por amostra** (`decide()`; ordem = prioridade):

```
se IMU inválido                         -> INVALID   (reset do portão, sem alerta)
seat = soma dos 4 FSRs do assento (cru)
se seat < EMPTY_RAW                     -> EMPTY     (ninguém sentado: reset, sem alerta)
rear    = rear_ok ? (f[seat_rear_left] + f[seat_rear_right]) / base_rear : 1
slump   = rear_ok e rear < SLUMP_FRAC
recline = rear_ok e droll > 0 e rear >= RECLINE_FRAC
dev     = hypot(dpitch, recline ? 0 : droll)          # para trás com quadril atrás: ignorado
lean    = lr_ok e |dpitch| >= LEAN_DEG e
          desloc >= LEAN_LR, onde lr = (fL - fR)/seat - base_lr
                                desloc = dpitch < 0 ? lr : -lr   # dpitch < 0 = inclinado à esquerda
se slump                -> SLUMP   (ruim)
senão se dev > ALERT_DEG -> IMU_DEV (ruim)
senão se lean           -> LEAN    (ruim)
senão                   -> RECLINE_OK se recline e hypot(dpitch, droll) > ALERT_DEG, senão OK
ok (pode limpar) = não ruim e dev < CLEAR_DEG
```

**Portão temporal** (mesma semântica do `AlertGate`): ruim continuamente por
`HOLD` → alerta; com alerta ativo, uma amostra `ok` limpa; nem ruim nem ok
mantém o estado e reinicia a contagem do `HOLD`. `EMPTY`, `INVALID` e uma nova
calibração zeram o portão.

| constante | valor | justificativa (dados) |
|---|---|---|
| `ALERT_DEG` / `CLEAR_DEG` | 10° / 7° | herdados (`analysis/archive/VALIDATION.md`); na grade, 9°, 10° e 11° empatam (0,936 / 0,935 / 0,936); 6/8 dobras LOSO escolhem 10° |
| `HOLD` | 3 s | herdado; 2 s / 3 s / 5 s dão 0,937 / 0,935 / 0,932 (plano) |
| `SLUMP_FRAC` | 0,3 | `escorregado`: mediana do assento traseiro 0 em 11/13 segmentos, 0,23 no Thiago-Fita (passada 2) e 0,49 no Gustavo (pego pelo IMU); nos outros segmentos p5 ≥ 0,52, exceto 3 escorregados de fato (ver falhas). 7/8 dobras LOSO |
| `RECLINE_FRAC` | 0,7 | `reclinado`: assento traseiro p5 0,65 agrupado (mediana 0,98); precisa ficar acima do `escorregado` parcial do Gustavo (0,49). 7/8 dobras LOSO |
| `LEAN_DEG` / `LEAN_LR` | 5° / 0,08 | ereto: \|dpitch\| ≤ 3,9° (p1–p99), deslocamento L/R do assento ≈ 0 (\|mediana\| ≤ 0,01); 6/8 dobras LOSO |
| `EMPTY_RAW` | 2000 | sentado, soma do assento ≥ 5222 em toda amostra assentada (mín. = `escorregado`, só a frente); vazio ≈ 0 |
| `BASE_MIN` | 1000 | bases do assento traseiro 4007–5999 em todas as calibrações registradas; as de antes do log feitas com a cadeira vazia (Andre, Gustavo) dão 0–2 |

Só somas de grupos do assento são usadas: as costas têm canais que ficam ≈ 0
mesmo ereto (back_lower de Thiago-M, Rafael, Ryan), então razões por canal
seriam frágeis. *Creep*: no ereto a razão do assento traseiro fica em
0,96–1,09 (p1–p99) ao longo de runs de 6–24 min, longe de 0,3 e 0,7.
Custo: ~20 operações de ponto flutuante e um `hypot` por amostra.

## Avaliação

**Método.** Replay amostra a amostra de cada run inteira (`FusionRule`,
igual ao MCU), com a linha de base reconstruída das 20 linhas da janela de
captura de cada `# calibrated` (19 antes do evento + a primeira com o novo
`calid`; a janela reproduz a referência do IMU registrada com erro ≤ 0,05°).
Toda postura rotulada está sob uma calibração feita dentro do log; para o
`calid` herdado de antes do log (8 das 9 runs) usam-se as primeiras 20 linhas
com ele — só afeta trechos não rotulados, e em Andre e Gustavo a cadeira
estava vazia (base ≈ 0, as guardas desligam os termos do assento). Pontuação
nas amostras assentadas (tag ≠ 0, descartados 2 s após cada troca de tag).
Sanidade: a regra atual re-executada concorda com a coluna `alert` gravada em
99,7–100 % das linhas (diferenças = arredondamento de 0,1° no limiar).
`8`/`9` contam como falso alarme. Critério de escolha: média de (fração
alertada dos segmentos ruins) e (fração quieta dos segmentos 1/7/8/9).

**LOSO.** Grade de 2250 candidatos (`ALERT_DEG` 8–12, `SLUMP_FRAC`
off/0,2–0,5, `RECLINE_FRAC` 0,5–0,9/off, `LEAN_DEG` 5–8/off, `LEAN_LR`
0,03/0,05/0,08; `CLEAR = ALERT − 3`). A pontuação é plana em platôs largos,
então a escolha usa a média com os vizinhos da grade (centro do platô).
Escolhe-se com 7 sujeitos e testa-se no 8º (Andre: run1 + run2 juntos).

| | segmentos ruins disparados | tempo ruim alertado | tempo ok (1, 7) alertado | tempo pernas (8, 9) alertado | pontuação |
|---|---|---|---|---|---|
| atual | 55/62 (89 %) | 84 % | 29 % (9/39 segm.) | 10 % | 0,834 |
| fusão (escolhida com os 8) | 58/62 (94 %) | 93 % | 2 % (1/39) | 11 % | 0,935 |
| **fusão, LOSO** | **58/62 (94 %)** | **92 %** | **3 % (2/39)** | **28 %** | **0,901** |

A regra atual não tem parâmetro ajustado nestes dados, então seus números já
são "fora da amostra". Pior sujeito: Vinicius (0,67 atual → 0,74 fusão/LOSO)
e Felipe (0,68 → 0,76 / 0,72 LOSO). Referência: a LDA LOSO grosseira dava
70 %/86 % (só IMU) e 76 %/95 % (IMU+FSR) de sensibilidade/especificidade;
aqui, LOSO, 92 % / 97 % (1, 7) por amostra.

**Por postura** (% do tempo assentado com alerta; fusão = parâmetros finais):

| tag | atual | fusão | LOSO | Andre | Felipe | Gustavo | Rafael | Ryan | T-Fita | T-M | Vinicius |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 ereto | 0 | 0 | 0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 | 0/0 |
| 2 curvado_frente | 100 | 100 | 100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 |
| 3 cabeca_frente | 70 | 70 | 70 | 81/81 | 100/100 | 0/0 | — | 100/100 | 100/100 | 0/0 | 0/0 |
| 4 escorregado | 72 | 100 | 94 | 94/100 | 100/100 | 100/100 | 0/100 | 92/100 | 47/100 | 100/100 | 100/100 |
| 5 inclinado_esq | 94 | 100 | 100 | 100/100 | 0/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 |
| 6 inclinado_dir | 87 | 93 | 94 | 86/99 | 100/100 | 100/100 | 100/100 | 81/93 | 100/100 | 100/100 | 38/38 |
| 7 reclinado | 78 | 5 | 7 | 81/0 | 100/100 | 100/0 | — | 99/0 | 52/0 | 0/0 | 100/0 |
| 8 perna_e_sobre_d | 6 | 12 | 21 | 0/0 | 91/91 | 0/0 | 0/24 | 0/0 | 0/0 | 0/0 | 0/0 |
| 9 perna_d_sobre_e | 14 | 10 | 35 | 0/0 | 0/0 | 100/0 | 0/0 | 1/1 | 0/0 | 0/0 | 69/100 |

Colunas por sujeito: atual/fusão. Latência: quase sempre 0 s a partir da
tecla da tag — a pessoa já está na postura durante a transição (tag `0`), e o
alerta dispara ali.

**Ablação** (pontuação, parâmetros finais): atual 0,834 → + vazio 0,834 →
+ `SLUMP` 0,850 → + `RECLINE` 0,922 → + `LEAN` 0,935. O ganho principal é não
alertar no `reclinado` (29 % → 2 % do tempo ok alertado); o `SLUMP` leva o
`escorregado` a 100 %; o `LEAN` recupera o `inclinado_esq` pequeno do Felipe
(dpitch −6°).

## Falhas conhecidas

- **`cabeca_frente` (3/8 sujeitos em 0 %):** Gustavo, Thiago-M e Vinicius
  dão só −4,6 a −7,5° de droll (medianas) e as costas continuam no encosto (FSRs iguais ao
  ereto). Baixar o limiar para frente esbarra no ereto do Rafael (droll p5
  −7,5°). Limite do IMU no colar; os FSRs não ajudam.
- **Deriva do IMU (Felipe):** dpitch ≈ +10° após o `6` → `reclinado` (100 %)
  e `perna_e_sobre_d` (91 %) alertam por `IMU_DEV`. O assento não mostra nada
  — é deriva do colar, não postura. Mitigação futura: recalibrar ou
  compensar deriva lenta quando o padrão dos FSRs bate com a linha de base.
- **`inclinado_dir` do Vinicius (38 %):** dpitch ~10,5° (no limiar) e o
  assento desloca para o lado errado (−0,04); só as costas mostram (lado
  esquerdo do encosto zera), mas os canais das costas não são confiáveis.
- **Posturas mistas:** `9` do Vinicius escorregado para frente (assento
  traseiro = 0 → `SLUMP`, 100 %, fisicamente correto, conta como falso
  alarme); `5` do Ryan na passada 2 ainda escorregado (`SLUMP`, acerto).
- **Margens finas que aparecem no LOSO:** `8` do Rafael (24 %): a perna
  cruzada desloca o assento (+0,15) com dpitch −5,7° → `LEAN`; `9` do
  Thiago-Fita tem assento traseiro 0,40 (dobra que escolhe `SLUMP_FRAC` 0,5
  alerta 100 %); `escorregado` do Gustavo (0,49) cai para 23 % na dobra que
  escolhe `RECLINE_FRAC` 0,5; a dobra sem o Andre escolhe `ALERT_DEG` 8° e
  alerta nas pernas cruzadas dele (deriva de −8° em dpitch na run2).
- **Histerese herdada:** um alerta de uma postura ruim pode atravessar a
  transição e continuar no início da próxima se o desvio ficar entre 7° e
  10°; "disparou no segmento" credita isso.

## Pernas cruzadas (decisão em aberto)

Pelos FSRs do assento (medianas assentadas, vs. ereto): o deslocamento L/R
passa de 0,1 em só 3/8 sujeitos no `8` (Rafael +0,15, Thiago-Fita +0,29,
Vinicius +0,12; Andre muda só a frente, 0,72) e 3/8 no `9` (Thiago-M −0,40,
Thiago-Fita e Vinicius pelo assento traseiro 0,40 / 0); o sinal não é
consistente entre pessoas (Thiago-M vai levemente para o outro lado no `8`, −0,07). No ereto
os mesmos índices ficam em ±0,03. Conclusão: **detectável em ~metade das
pessoas, sem um padrão comum** com 4 FSRs no assento; a regra não alerta
nelas (8 e 9 ficam em 12 % / 10 % do tempo: deriva do Felipe, `LEAN` do
Rafael e o escorregamento real do Vinicius). Decidir se pernas cruzadas devem alertar
exige mais dados (e talvez outro arranjo de sensores no assento).

## Ninguém sentado

Soma do assento < 2000 → `EMPTY`: nunca alerta e zera o portão. Nas 9 runs há
176 s de cadeira vazia (em pé com o colar); a regra atual alertou em 71 s
deles, a fusão em 0. É também um sinal de presença barato (pausar alertas,
contar tempo sentado).

## Reproduzir

```bash
.venv/bin/python analysis/fusion.py           # tabelas + LOSO (~1 min)
.venv/bin/python analysis/fusion.py --quick   # sem a grade LOSO
```
