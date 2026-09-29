# Pesquisa: bateria LiPo 1S da unidade corporal × XIAO nRF52840 Sense

Pesquisa web feita em 2026-09-29. Não há datasheet público do pack exato; os
itens marcados *típico* vêm de packs equivalentes e **precisam ser medidos**.

## Bateria

Etiqueta: `BAT04210903176` · `JYH LIP103450-1800 1S1P` · `3.7V 1800mAh 6.6Wh`

| Item | Valor | Fonte |
|---|---|---|
| Fabricante | JYH Technology (Jiangmen, China) | listagem JYH |
| Dimensões ("103450") | 10 × 34 × 50 mm | listagem JYH |
| Tensão de fim de carga | 4,2 V (célula comum, não HV 4,35 V) | listagem JYH |
| Carga padrão / máx. | 900 mA (0,5C) / 1800 mA (1C) | listagem JYH |
| Tensão de corte na descarga | 2,75 V | listagem JYH |
| Temperatura de carga | 0–45 °C | listagem JYH |
| 3º fio | **NTC 10 kΩ** entre amarelo/branco e preto (−); B = 3435 ou 3950 | *típico* |
| Cores | vermelho = +, preto = − | *típico* |
| Proteção (PCM) | opcional na JYH — **não confirmado** nesta unidade | listagem JYH |
| Conector | geralmente JST-PH 2,0 mm; às vezes 1,25 mm (PicoBlade), 1,0 mm (JST-SH) ou 2,54 mm (JST-XH) | *típico* |

A polaridade/ordem dos pinos em conectores JST **não é padronizada**.

## Carregador do XIAO nRF52840 Sense

- **CI de carga:** BQ25101 (o símbolo no esquemático v1.1 diz BQ25100, mas a
  ligação corresponde ao 101).
- **Ligação:** apenas **pads BAT+ / BAT− no verso**; não há conector.
- **Corrente de carga** (ISET: 2,7 kΩ ao GND + 2,7 kΩ ao P0.13):
  - P0.13 como entrada flutuante → **50 mA** (padrão);
  - P0.13 em **LOW** → **100 mA**;
  - não levar P0.13 a HIGH.
- **Termistor:** o pino TS do carregador está preso a 10 kΩ fixo ("temp sense
  desabilitado") e não é exposto → **o 3º fio da bateria não tem onde ser
  ligado no XIAO**; deixá-lo solto não perde nada.
- **LED de carga:** P0.17 em LOW = carregando; HIGH = terminou *ou* não está
  carregando.
- **Uso com USB conectado:** a placa roda do USB (chave PMOS + diodo), enquanto
  a bateria só carrega.

### ⚠️ Timer de segurança — limita a carga desta bateria

O BQ25101 tem um pré-carga de 30 min e um **timer de carga rápida de ~10 h**
(9,4–12,5 h). Ao expirar, a carga **para e o LED apaga**; ela só recomeça ao
replugar o USB ou reiniciar a alimentação.

- 1800 mAh a 100 mA → ~18–20 h → uma conexão chega a **~55 %**.
- A 50 mA (padrão) → **~28 %**.
- **"LED apagado" não significa bateria cheia.**

### ⚠️ Leitura da tensão da bateria (para o firmware futuro)

A tensão da bateria é lida em P0.31 (AIN7) por um divisor 1 MΩ / 510 kΩ
(Vbat ≈ 2,96 × Vadc). O divisor é habilitado com **P0.14 em LOW**.
**Nunca levar P0.14 a HIGH:** isso pode colocar ~3,6 V em P0.31 e danificar o
pino, principalmente durante a carga.

## Conclusão

**Ligar só + e − nos pads BAT+/BAT− e carregar pelo USB-C é seguro e
funciona.** O 3º fio (NTC) fica isolado. Ressalvas:

- Conferir a polaridade com multímetro antes de ligar; inverter danifica a placa.
- A célula fica sem monitoramento de temperatura (o XIAO não tem onde usar o NTC).
- Não se sabe se o pack tem PCM.
- **A carga pelo XIAO é impraticável para 1800 mAh:** exige P0.13 = LOW no
  firmware e replugar o USB, ou aceitar carga parcial. O lado bom é uma taxa
  muito suave (≈0,06C).

Alternativas, se o tempo de carga importar (**nunca** usar dois carregadores
na célula ao mesmo tempo):

- **Módulo TP4056 + DW01:** 1 A de fábrica, ajustável por R_PROG; muitos
  ignoram o NTC.
- **MCP73831 (ex.: Adafruit Micro-Lipo):** até 500 mA, sem NTC.
- **Adafruit bq24074:** até ~1,5 A, usa NTC 10 kΩ e tem power-path.

## Conector

**Medir antes de comprar:**
- Passo entre centros dos pinos: 2,0 mm = JST-PH, 1,25 mm = PicoBlade,
  1,0 mm = JST-SH, 2,54 mm = JST-XH. Um JST-PH de 3 vias tem ~8 mm de largura.
- Vermelho ↔ preto: deve dar 3,0–4,2 V.
- 3º fio ↔ preto: ~10 kΩ a 25 °C, caindo ao aquecer a célula com a mão
  (confirma NTC).

**O que buscar** (se for PH): "JST PH 2.0 3 pinos macho com cabo/rabicho".
Quando chegar, conferir de novo qual fio é o + em relação à bateria antes de
soldar, porque as cores do rabicho frequentemente não seguem a ordem da
bateria.

## Fontes

- Listagens JYH: <https://jyhtechnology.en.made-in-china.com/product/UOjEkhJlCHWb/China-Lip103450-1800-1s1p-High-Capacity-3-7V-1800mAh-Li-Polymer-Rechargeable-Battery-Cell.html>,
  <https://jyhtechnology.en.made-in-china.com/product/cdBJfWEHZlkZ/China-Lip103450-3-7V-1800mAh-8-66wh-Rechargeable-Li-Polymer-Battery-Pack.html>
- Packs 103450 equivalentes: <https://mit.ek.dk/media/5m1o2o1k/lp103450-1800mah-datasheet.pdf>,
  <https://www.dnkpower.com/wp-content/uploads/2023/11/DNK103450-3.7V-1800mAh-Lipo-Battery-Specification.pdf>,
  <https://www.topwellpower.com/products/twe-103450-37v-1800mah-li-polymer-battery-with-ntc>
- Seeed: <https://wiki.seeedstudio.com/XIAO_BLE/>,
  <https://wiki.seeedstudio.com/battery_charging_considerations/>,
  esquemático <https://files.seeedstudio.com/wiki/XIAO-BLE/Seeed-Studio-XIAO-nRF52840-Sense-v1.1.pdf>
- TI BQ25101 (timer de segurança, §8.3.9): <https://www.ti.com/lit/ds/symlink/bq25101.pdf>
- Problema do P0.14: <https://github.com/honvl/Seeed-Xiao-NRF52840-Battery/issues/1>
- Carga que "trava" em 3,7 V: <https://forum.seeedstudio.com/t/xiao-nrf52840-sense-charging-consistently-stalls-at-3-7v-never-reaches-4-2v/295688>
