# Tarefas pendentes

Lista de coisas a fazer que ainda não viraram trabalho ativo. Cada item tem
contexto suficiente para ser retomado a frio. Quando concluir, marque `[x]` e
registre o resultado (ou mova a conclusão para o doc/CLAUDE.md pertinente).

Decisões de projeto em aberto continuam em `CLAUDE.md` → *Open questions*;
aqui ficam as ações concretas.

## Hardware — unidade corporal

### [ ] Ligação da bateria LiPo 1S (3 fios) ao XIAO nRF52840 Sense

**Contexto:** a bateria comprada é de **uma célula (1S, 3,7 V)** mas tem
**3 fios** e um **conector de 3 vias**. Objetivo: alimentar o XIAO por ela e
recarregá-la pelo USB-C da própria placa, **sem cortar os fios da bateria**.

**Hipóteses a verificar (não confirmadas):**
- O 3º fio costuma ser um **termistor NTC** (tipicamente 10 kΩ a 25 °C, entre
  o fio e o negativo) ou, mais raramente, um pino de identificação. Se for NTC,
  deixá-lo desconectado deve ser aceitável — o XIAO não usa termistor.
- O XIAO tem pads **BAT+ / BAT−** no verso e um carregador onboard (BQ25101)
  que carrega a célula pelo USB-C (~50 mA padrão; ~100 mA configurável via
  pino P0.13). Então **só + e − ligados nos pads + carga via USB-C** deve
  funcionar. Confirmar no wiki/esquemático da Seeed.

**Passos:**
- [ ] Ler a etiqueta/datasheet da bateria (capacidade em mAh, se tem circuito
      de proteção/PCM embutido, função do 3º fio).
- [ ] Identificar o conector: medir o passo entre pinos (2,0 mm → provável
      JST-PH; 1,25 mm → Molex PicoBlade/JST-GH; 1,0 mm → JST-SH) e fotografar.
- [ ] Com multímetro: tensão entre os fios para achar + e − (**polaridade de
      conectores JST não é padronizada**); resistência do 3º fio ao − para
      confirmar se é NTC.
- [ ] Verificar se a corrente de carga do XIAO (50/100 mA) é adequada à
      capacidade da célula (carga ≤ ~1C; para células pequenas, 50 mA é seguro).
- [ ] Comprar o **conector fêmea correspondente com rabicho** (pigtail) para
      ligar aos pads BAT+/BAT− sem cortar a bateria.
- [ ] Testar: alimentar pela bateria, conectar USB-C e confirmar que carrega
      (LED de carga do XIAO) e que a placa roda só na bateria.
- [ ] Registrar o resultado no `CLAUDE.md` (seção de energia / bateria).
