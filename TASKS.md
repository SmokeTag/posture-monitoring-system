# Tarefas pendentes

Lista de coisas a fazer que ainda não viraram trabalho ativo. Cada item tem
contexto suficiente para ser retomado a frio. Quando concluir, marque `[x]` e
registre o resultado (ou mova a conclusão para o doc/CLAUDE.md pertinente).

Decisões de projeto em aberto continuam em `CLAUDE.md` → *Open questions*;
aqui ficam as ações concretas.

## Hardware — unidade corporal

### [ ] Ligação da bateria LiPo 1S (3 fios) ao XIAO nRF52840 Sense

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
- [ ] Registrar o resultado no `CLAUDE.md` (seção de energia / bateria).
