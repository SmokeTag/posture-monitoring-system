# Guia — `analysis/mostra.py` (versão para a mostra de ciência)

Tela cheia para o público, uma pessoa por vez e muitas pessoas por dia. Lê o
mesmo stream da cadeira que o `live_posture.py`, mas sem diagnóstico: o
diagnóstico continua no `live_posture.py` (feche um antes de abrir o outro,
porque a porta serial é uma só).

```bash
.venv/bin/python analysis/mostra.py              # abre em tela cheia (f alterna)
.venv/bin/python analysis/mostra.py --window     # em janela
.venv/bin/python analysis/mostra.py --replay data/validation/2026-10-22_Andre-Monteiro_run1.csv
```

`--replay` toca uma captura gravada no ritmo original, em loop: serve para
testar sem o hardware e como plano B se algo falhar na mostra. Os números do
dia de um replay ficam só na memória (não contam no dia real).

## O que aparece

- **Faixa de status** (topo), sempre com ícone e texto, não só cor:
  - verde `✓ Postura boa`;
  - âmbar `● Atenção` (postura ruim, ainda sem vibrar);
  - vermelho `▲ Corrija a postura` (o colar está vibrando).

  A faixa diz o nome da postura (curvado para a frente, inclinado para a
  esquerda/direita, escorregado no assento; reclinado é ok). Em cinza, o que
  falta fazer: ligar o colar, sentar, calibrar.
- **Sua coluna:** o desvio da postura calibrada (centro), com a "zona boa"
  (10°) e, em cinza, onde caíram as posturas gravadas em 21/10.
- **Sua cadeira:** os 8 FSRs como pressão, o centro do peso (`+`) em cada placa
  e a divisão esquerda/direita do assento.
- **Como funciona:** colar, cadeira, decisão e calibração, em 4 passos.
- **Você agora / Última pessoa:**
  - tempo sentado;
  - % de postura boa;
  - alertas (vibrações);
  - maior sequência boa;
  - linha do tempo dos últimos 2 min.
- **Hoje na mostra:** pessoas, média de postura boa e recorde do dia.

## Fluxo com o visitante

1. Coloque o colar no visitante e peça que ele se sente reto, com as costas no
   encosto.
2. Aperte o **botão da cadeira** (ou `c` / espaço no notebook). A faixa mostra
   "Calibrando… fique parado" e depois "Pronto!". **A visita começa aqui.**
   Recalibrar sem levantar continua a mesma visita.
3. Mostre as posturas: curvar, inclinar, escorregar. Reclinar e cruzar as
   pernas não alertam.
4. Quando o visitante levanta e o assento fica **vazio por 10 s** (ou ao apertar
   `n`), a visita termina:
   - os números dela ficam como "Última pessoa";
   - ela entra em "Hoje na mostra", se durou ≥ 15 s sentada;
   - a calibração da cadeira é **apagada**, para o próximo não levar vibração
     com a referência de outra pessoa. `--keep-cal` desliga isso.

Os números do dia ficam em `~/.local/state/tcc-mostra/<data>.json` (`--stats-dir`),
então reabrir o app não zera o dia.

## Teclas

| tecla | ação |
|---|---|
| `c` / espaço | calibrar (o mesmo que o botão da cadeira) |
| `n` | próximo visitante (encerra a visita agora) |
| `r` | apagar a calibração |
| `f` | tela cheia |
| `q` | sair |

O rodapé mostra, pequeno, a bateria do colar e o enlace, para o operador.
