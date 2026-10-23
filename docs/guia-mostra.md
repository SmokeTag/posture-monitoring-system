# Guia — `analysis/mostra.py` (versão para a mostra de ciência)

Tela cheia para o público, uma pessoa por vez e muitas pessoas por dia. Lê o
mesmo stream da cadeira que o `live_posture.py`, mas sem diagnóstico: o
diagnóstico continua no `live_posture.py` (feche um antes de abrir o outro,
porque a porta serial é uma só).

```bash
.venv/bin/python analysis/mostra.py              # USB com BLE automático (f alterna tela cheia)
.venv/bin/python analysis/mostra.py --window     # em janela
.venv/bin/python analysis/mostra.py --replay data/validation/2026-10-22_Andre-Monteiro_run1.csv
```

`--replay` toca uma captura gravada no ritmo original, em loop: serve para
testar sem o hardware e como plano B se algo falhar na mostra. Os números do
dia de um replay ficam só na memória (não contam no dia real).

## Cabo e BLE automático

O comando sem opções prioriza o cabo USB. Se ele desconectar ou ficar 1 s sem
dados válidos, o painel passa a usar BLE; ao voltar a receber dados pelo cabo,
retorna ao USB. O rodapé indica a conexão atual, e a visita e seus contadores
continuam na mesma janela. O Bluetooth precisa estar ligado para o fallback.
Se ambos ficarem sem sinal, aparece "Conectando à cadeira…"; a troca depende
de chegar uma nova amostra e continua sujeita às pausas de recepção do BLE.

Os dois leitores ficam ativos, mas só uma fonte alimenta o painel por vez.
O modo automático usa o relógio do notebook para a linha do tempo, pois o BLE
transmite só parte do relógio da cadeira. As estatísticas continuam estimadas
entre amostras; dados perdidos não são recuperados.

Para forçar uma conexão: `--usb` usa só o cabo; `--ble` usa só BLE.
`--ble-address AA:BB:CC:DD:EE:FF` seleciona a cadeira no BLE, inclusive no
modo automático. Com várias cadeiras próximas, informe o endereço correto.

## Sem fio (BLE)

Ligue o Bluetooth do notebook (`rfkill unblock bluetooth; bluetoothctl power on`),
alimente cadeira e colar e execute:

```bash
.venv/bin/python analysis/mostra.py --ble
```

Calibre pelo botão da cadeira, sentado reto. O enlace é só de leitura:
sem host USB conectado, a cadeira apaga a calibração após 10 s de assento vazio.
Com USB aberto, a limpeza continua manual. Em BLE, as teclas `c` / espaço e `r`
não enviam comandos nem os deixam pendentes para quando o cabo voltar.

No ensaio de 60 s: 20 beacons/s enviados, 3,75/s recebidos (18,75%);
intervalo máximo 1,9 s, sem perda ESB. O painel preenche intervalos de até 2 s;
a atualização pode variar. No modo automático basta conectar o cabo para
retomar o USB; com `--ble`, reinicie sem essa opção para usar o cabo.

Opcional, para investigar melhor recepção: habilitar `[General] Experimental=true`
em `/etc/bluetooth/main.conf` e reiniciar o serviço BlueZ (requer administrador),
depois adaptar o leitor para scan passivo bleak com `or_patterns`.
Não foi habilitado/testado neste notebook (sem sudo); o leitor atual usa scan ativo.

## O que aparece

- **Faixa de status** (topo), sempre com ícone e texto, não só cor:
  - verde `✓ Postura boa`;
  - âmbar `● Atenção` (postura ruim, ainda sem vibrar);
  - rosa `▲ Corrija a postura` (o colar está vibrando).

  Cores e fonte seguem a identidade do stand ([`stand/README.md`](stand/README.md)).
  Os estados neutros (ligar o colar, sentar, calibrar) usam o gradiente azul →
  violeta → magenta dos painéis.

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
| `c` / espaço | calibrar pelo USB (no BLE, use o botão da cadeira) |
| `n` | próximo visitante (encerra a visita agora) |
| `r` | apagar a calibração pelo USB |
| `f` | tela cheia |
| `q` | sair |

O rodapé mostra, pequeno, a bateria do colar e o enlace, para o operador.
