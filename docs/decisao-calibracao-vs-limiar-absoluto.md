# Decisão de projeto: calibração por usuário vs. limiar absoluto

> **Status:** raciocínio preliminar — inclinação atual pela **calibração por
> usuário (botão de calibração)**, ainda a ser validado experimentalmente antes
> de fixar a decisão. Documento de apoio para o TCC.

## Contexto

A unidade de corpo (XIAO nRF52840 Sense, usada na parte superior das costas,
logo abaixo do pescoço) estima a orientação do tronco a partir do vetor de
gravidade medido pelo acelerômetro do IMU (LSM6DS3TR-C). Dessa medida derivam-se
os ângulos de inclinação:

- **pitch** — inclinação para frente/trás (eixo do *slouch*, debruçar sobre a
  mesa);
- **roll** — inclinação para a esquerda/direita.

A pergunta de projeto é: **como definir o que é postura "correta" vs.
"problemática"?** Há duas estratégias possíveis:

1. **Limiar absoluto (fixo):** disparar o alerta quando o ângulo absoluto
   ultrapassar um valor pré-definido em código (ex.: pitch > 30°).
2. **Calibração por usuário:** o usuário senta-se ereto e registra a postura
   atual como referência ("ereto"); o sistema passa a alertar com base no
   **desvio** em relação a essa referência (dPitch/dRoll), e não no ângulo
   absoluto.

## Observação-chave

O acelerômetro **já fornece orientação absoluta em relação à gravidade**, sem
deriva (*drift*) — diferentemente da integração do giroscópio. Portanto, o
problema do limiar absoluto **não é deriva nem falta de referência física**. O
que a calibração realmente remove são **dois deslocamentos (offsets)** que um
limiar fixo não consegue tratar:

### 1. Offset de montagem (como o sensor assenta nas costas)

A leitura absoluta depende inteiramente de como a placa é fixada ao corpo a cada
uso. Isso foi observado diretamente durante os testes de bring-up: para a mesma
pessoa em repouso, em sessões diferentes, a postura de referência mudou de forma
significativa apenas porque a placa assentou de modo distinto:

| Sessão | pitch em repouso | roll em repouso |
|--------|------------------|-----------------|
| A      | ≈ 47°            | ≈ −49°          |
| B      | ≈ 43°            | ≈ −3°           |

A variação de roll (de −49° para −3°) entre montagens é maior do que muitos dos
desvios posturais que se deseja detectar. Um limiar absoluto fixo "passaria a
significar coisas diferentes" a cada vez que o dispositivo fosse vestido.

### 2. Postura ereta individual

A postura ereta natural da parte superior das costas varia de pessoa para
pessoa. Um limiar que represente "curvado" para um indivíduo pode corresponder à
postura normal de outro. Um valor único em código não atende ao objetivo de
adaptação por usuário.

A calibração colapsa **ambos** os deslocamentos em um único gesto (apertar o
botão enquanto sentado ereto).

## Quando o limiar absoluto funcionaria

Um limiar absoluto só seria robusto se houvesse, simultaneamente:

- uma **montagem rígida e repetível** (suporte/bolso que posicione a placa de
  forma idêntica a cada uso), eliminando o offset de montagem; e
- a aceitação de uma **definição única de postura** para todos os usuários,
  abrindo mão da adaptação individual.

Ambas as condições contrariam premissas do projeto (uso real, vestir/retirar
repetidamente, adaptação por pessoa), o que torna o limiar absoluto frágil neste
cenário.

## Atrito da calibração e como mitigá-lo

O incômodo percebido não está na calibração em si (um toque de botão), mas em
**ter de repeti-la**. A mitigação é **persistir a referência na memória flash**
do nRF52840 (ex.: `Adafruit_LittleFS` / `InternalFileSystem`), de modo que a
calibração sobreviva a reinicializações e se torne, de fato, **uma única vez por
usuário** — e não a cada *boot*.

## Caminho intermediário (opcional, futuro)

Para um cenário "sem botão", seria possível uma **autocalibração oportunista**:
assumir que os primeiros segundos após sentar correspondem à postura ereta, ou
rastrear a postura "mais ereta" observada em uma janela de tempo. É uma
abordagem **frágil** (se a pessoa já se sentar curvada, a referência fica
errada), portanto recomenda-se mantê-la apenas como camada de conveniência,
preservando a calibração explícita como caminho principal.

## Inclinação atual

- **Adotar a calibração por usuário** via botão de calibração (planejado na
  unidade da cadeira / MCU da cadeira, que já possui o enlace de rádio ESB com o
  cérebro — o XIAO não tem botão de usuário utilizável, apenas RESET).
- **Adicionar persistência em flash** para que a calibração seja única por
  usuário.
- **Validar experimentalmente** antes de fixar a decisão: comparar a
  estabilidade da detecção com limiar absoluto vs. desvio calibrado em
  repetições de vestir/retirar e entre diferentes pessoas.

## Estado de validação

Calibração já comprovada em firmware (`firmware/xiao_imu_test/`): o comando
serial `c` captura a média de 20 amostras como referência "ereto" e o sketch
passa a exibir o desvio (dPitch/dRoll). Resta a comparação experimental
sistemática descrita acima.
