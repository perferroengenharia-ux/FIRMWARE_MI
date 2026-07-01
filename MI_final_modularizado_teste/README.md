# MI modular - STM32F301C8T6 / NUCLEO-F301C8T6

## Objetivo

Este pacote separa o firmware do MI em módulos `.c/.h`, mantendo o `main.c` gerado pelo CubeMX o mais limpo possível.

Base preservada do firmware funcional atual:

- Protocolo RS485 com a IHM.
- Lógica antiga/funcional de acionamento de motor, bomba, swing e dreno.
- Sensores por ADC fora da ISR.
- Agendamento de sensores via timer lento.
- SPWM por TIM de interrupção separado.

## Arquivos

Copie os arquivos para o projeto CubeIDE:

```text
Core/Inc/mi_config.h
Core/Inc/mi_types.h
Core/Inc/mi_platform.h
Core/Inc/mi_state.h
Core/Inc/mi_storage.h
Core/Inc/mi_periph.h
Core/Inc/mi_sensors.h
Core/Inc/mi_motor.h
Core/Inc/mi_comm.h
Core/Inc/mi_app.h

Core/Src/mi_platform.c
Core/Src/mi_state.c
Core/Src/mi_storage.c
Core/Src/mi_periph.c
Core/Src/mi_sensors.c
Core/Src/mi_motor.c
Core/Src/mi_comm.c
Core/Src/mi_app.c
```

O arquivo `Core/Src/main_user_blocks.c` não é para compilar como arquivo normal. Ele contém os blocos para copiar para o `main.c` gerado pelo CubeMX.

## Configuração CubeMX sugerida

### MCU / Board

- Board: `NUCLEO-F301C8T6`
- MCU: `STM32F301C8T6`
- Clock recomendado para manter compatibilidade direta: `48 MHz`

Se usar outro clock, ajuste prescalers dos timers para manter base de 1 MHz.

### TIM1 - PWM complementar

Usado para as três fases do inversor.

- TIM1 PWM Generation CH1 + CH1N
- TIM1 PWM Generation CH2 + CH2N
- TIM1 PWM Generation CH3 + CH3N
- Prescaler: `48-1` com clock de timer de 48 MHz
- Period inicial: `100-1`
- Dead time: ajuste conforme seu hardware. Valor equivalente ao usado antes: ~2 us.
- Auto-reload preload: Enable

### TIM2 - interrupção do SPWM

Substitui o TIM3 usado na placa anterior.

- TIM2 Base Timer
- Prescaler: `48-1`
- Period inicial: `100-1`
- NVIC TIM2 global interrupt: Enable

O firmware altera o ARR conforme P42: 5 kHz, 10 kHz, 15 kHz.

### TIM16 - scheduler 100 ms

Substitui o TIM14 usado na placa anterior.

- TIM16 Base Timer
- Prescaler: `48000-1`
- Period: `100-1`
- NVIC TIM16 global interrupt: Enable

Com clock de 48 MHz, gera interrupção a cada 100 ms.
A cada 10 interrupções o firmware executa a lógica de 1 segundo do sensor de nível.

### USART1 - RS485 com IHM

- USART1 Asynchronous
- Baud: `115200`
- Word Length: `9 Bits`
- Parity: `Even`
- Stop Bits: `1`
- TX/RX conforme sua placa
- DMA RX: Enable
- NVIC USART1 global interrupt: Enable
- RS485_EN como GPIO output

No `stm32f3xx_it.c`, chame `mi_comm_uart_irq_handler(&huart1)` no início do `USART1_IRQHandler`.

### ADC1 - sensores

Neste pacote a leitura é feita canal a canal por polling, fora da ISR.
Configure o ADC de forma simples:

- ADC1 enabled
- Resolution: 12 bits
- Continuous Conversion: Disabled
- Scan Conversion: Disabled ou 1 conversão regular
- External Trigger: Software Start
- Sampling Time: alto, por exemplo `181.5 cycles`

Canais padrão em `mi_config.h`:

```c
#define MI_ADC_CH_CURRENT   ADC_CHANNEL_4
#define MI_ADC_CH_TEMP      ADC_CHANNEL_6
#define MI_ADC_CH_VBUS      ADC_CHANNEL_3
```

Ajuste `MI_ADC_CH_TEMP` caso o sensor de temperatura esteja em outro canal.

## Nomes obrigatórios dos GPIOs no CubeMX

Os módulos usam os nomes gerados no `main.h`. Nomeie os pinos assim:

```text
SENSOR_Pin / SENSOR_GPIO_Port
BOMBA_Pin / BOMBA_GPIO_Port
MOTOR_Pin / MOTOR_GPIO_Port
SWING_Pin / SWING_GPIO_Port
DRENO_Pin / DRENO_GPIO_Port
RS485_EN_Pin / RS485_EN_GPIO_Port
Led_Pin / Led_GPIO_Port
```

## Flash / parâmetros

O arquivo `mi_storage.c` usa `MI_FLASH_USER_ADDR = 0x0800F800` por padrão, reservado para STM32F301C8T6 com 64 KiB de flash.

Importante: reserve a última página no linker para evitar que o firmware ocupe a área de parâmetros.

## Observação sobre IHM / P44

O MI foi mantido como executor. Ele não tenta reconstruir P30/P31/P44 sozinho.
A restauração perfeita de secagem, climatizar/ventilar após erro e tempo restante de P31 deve ficar no IHM, pois o IHM controla os ciclos e estados.


## Ajustes importantes para evitar erros de build

- O arquivo `Core/Src/main_user_blocks.txt` é apenas um guia de cópia para o `main.c` gerado pelo CubeMX. Não renomeie para `.c` e não deixe como arquivo compilável.
- Use apenas `mi_periph.c`. Se existir um arquivo antigo chamado `mi_peripherals.c`, exclua-o do projeto ou remova-o do build.
- O código espera o LED com User Label `Led`. Se seu projeto antigo usa `ED`, há fallback em `mi_config.h`, mas o ideal é renomear para `Led` no CubeMX.
- `mi_comm.h` inclui `mi_config.h`, onde `MI_RX_DMA_BUF_SZ` é definido.
