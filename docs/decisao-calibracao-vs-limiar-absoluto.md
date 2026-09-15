# Decisão de projeto: calibração por usuário vs. limiar absoluto

> **Status:** RESOLVIDO — adotada a **calibração por usuário** (referência de
> "ereto" registrada por botão); o alerta dispara pelo **desvio** em relação a
> essa referência, não pelo ângulo absoluto. Validado experimentalmente
> (Campanha A / OE1). Documento de apoio para o TCC.

## Pergunta de projeto

Como definir o que é postura "correta" vs. "problemática" a partir da orientação
do tronco medida pelo IMU (acelerômetro do XIAO nRF52840 Sense, usado na parte
superior das costas)? Dessa medida derivam-se **pitch** (inclinação frente/trás,
eixo do *slouch*) e **roll** (inclinação esquerda/direita). Duas estratégias:

1. **Limiar absoluto (fixo):** alertar quando o ângulo absoluto ultrapassar um
   valor pré-definido em código (ex.: pitch > 30°).
2. **Calibração por usuário:** o usuário senta-se ereto e registra a postura
   atual como referência; o sistema alerta pelo **desvio** (dPitch/dRoll) em
   relação a ela, não pelo ângulo absoluto.

## Por que o limiar absoluto é frágil

O acelerômetro já fornece orientação absoluta em relação à gravidade, sem deriva
(*drift*) — logo o problema **não** é deriva nem falta de referência física. O
que a calibração remove são **dois deslocamentos (offsets)** que um limiar fixo
não consegue tratar:

1. **Offset de montagem** — a leitura absoluta depende inteiramente de como a
   placa assenta nas costas a cada uso (vestir/retirar repetidamente).
2. **Postura ereta individual** — o "ereto" natural varia de pessoa para pessoa;
   um valor único em código não se adapta a cada usuário.

A calibração colapsa **ambos** em um único gesto (apertar o botão sentado ereto).
Um limiar absoluto só seria robusto com montagem rígida e repetível **e** uma
definição única de postura para todos — duas condições que contrariam premissas
do projeto.

## Resultado experimental (OE1)

Campanha A (`data/2026-06-19_andre_no-neck-mount_own-chair.csv`; análise em
`analysis/VALIDATION.md`): em **4 re-dons genuínos** (vestir/retirar com nova
captura de referência), a postura "ereto" capturada **derivou 8.0° em pitch e
21.6° em roll** (`figures/04_redon_drift.png`). Essa variação entre montagens é
maior do que vários dos desvios posturais que se deseja detectar — ou seja, **um
limiar absoluto fixo passaria a significar coisas diferentes a cada vez que o
dispositivo fosse vestido**, confirmando a necessidade da calibração por usuário.

O mesmo conjunto mostra que o **desvio calibrado (dpitch/droll) é mais
reprodutível e mais separável** entre re-dons do que os ângulos absolutos —
portanto é o espaço de decisão adequado para o alerta.

## Decisão

- **Calibração por usuário** acionada por **botão na unidade da cadeira / MCU da
  cadeira** (que já possui o enlace de rádio ESB com o cérebro). Pressioná-lo 
  enquanto sentado ereto envia o comando de calibração; o XIAO de corpo 
  captura o pitch/roll atual como referência "ereto".
- **Alerta por desvio** em relação a essa referência, não por ângulo absoluto.
- **Persistir a referência em flash** (`Adafruit_LittleFS` /
  `InternalFileSystem`) para que a calibração sobreviva a reinicializações e
  seja, de fato, **uma vez por usuário** — não a cada *boot*. Isso remove o único
  atrito real (ter de repetir a calibração).

## Estado de implementação

Calibração comprovada em firmware (`firmware/xiao_imu_test/`): o comando serial
`c` captura a média de 20 amostras como referência "ereto" e o sketch passa a
exibir o desvio (dPitch/dRoll). Pendentes: botão físico na cadeira (hoje *stub*
pelo comando serial `c`), o enlace ESB e a persistência em flash.
