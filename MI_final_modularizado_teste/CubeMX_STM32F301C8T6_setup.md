# Passo a passo no STM32CubeMX para o MI em STM32F301C8T6

## 1. Criar o projeto
1. Abra o **STM32CubeMX**.
2. Crie um projeto novo para **STM32F301C8Tx / STM32F301C8T6**.
3. Em **Project Manager**, selecione **STM32CubeIDE** como toolchain.
4. Em **Code Generator**, habilite:
   - `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`
   - `Keep User Code when re-generating`

## 2. SYS / Debug
Em **System Core > SYS**:
- `Debug = Serial Wire`

> Isso libera pinos do antigo JTAG e facilita usar PB3/PB4/PB5 como GPIO, se desejar.

## 3. Clock
### Recomendação simples
Use **HSI + PLL** para chegar a **72 MHz**.

Em **Clock Configuration**:
- `HSI = 8 MHz`
- `PLL Source = HSI/2` (ou a combinação equivalente mostrada pelo CubeMX)
- ajuste a PLL para que `SYSCLK = 72 MHz`
- `AHB = 72 MHz`
- `APB1 = 36 MHz`
- `APB2 = 72 MHz`

## 4. Pinout recomendado
### Potência / PWM (TIM1)
- `PA8  -> TIM1_CH1`
- `PA9  -> TIM1_CH2`
- `PA10 -> TIM1_CH3`
- `PB13 -> TIM1_CH1N`
- `PB14 -> TIM1_CH2N`
- `PB15 -> TIM1_CH3N`

### RS485 (USART1)
- `PB6 -> USART1_TX`
- `PB7 -> USART1_RX`
- `PB8 -> GPIO_Output`  (nome: `RS485_EN`)

### Periféricos
Sugestão de GPIOs livres no LQFP48:
- `PB3  -> BOMBA`
- `PB4  -> MOTOR`
- `PB5  -> SWING`
- `PB10 -> DRENO`
- `PB0  -> SENSOR` (GPIO input)
- `PA15 -> STATUS_LED` (opcional)

> Se a sua PCB tiver pinagem diferente, pode remapear. O pacote modular usa os nomes do `main.h` gerado pelo CubeMX.

## 5. USART1
Em **Connectivity > USART1**:
- Mode: `Asynchronous`
- Baud rate: `115200`
- Word length: `9 Bits`
- Parity: `Even`
- Stop bits: `1`
- HW flow control: `None`

### DMA
Em **USART1 > DMA Settings**:
- adicione **RX DMA**
- use o canal que o CubeMX sugerir para `USART1_RX`

### NVIC
Habilite:
- `USART1 global interrupt`
- a interrupção do canal DMA usado em `USART1_RX`

## 6. TIM1 – PWM complementar do inversor
Em **Timers > TIM1**:
- habilite:
  - `PWM Generation CH1`
  - `PWM Generation CH2`
  - `PWM Generation CH3`
  - complementares `CH1N`, `CH2N`, `CH3N`

Parâmetros iniciais recomendados para manter a lógica do código:
- Prescaler = `71`
- Counter mode = `Up`
- Period = `99`
- Auto-reload preload = `Enable`
- Pulse inicial = `50`

> O código depois ajusta dinamicamente o ARR conforme `P42`.

## 7. TIM3 – base do SPWM
Em **Timers > TIM3**:
- Clock source = `Internal Clock`
- Prescaler = `71`
- Period = `99`
- habilite `TIM3 global interrupt`

## 8. TIM6 – tick de periféricos
Em **Timers > TIM6**:
- Mode = `Basic timer`
- Prescaler = `7199`
- Period = `9999`
- habilite `TIM6 global interrupt`

> Essa configuração gera um tick de aproximadamente 1 segundo, compatível com `mi_periph_tick_1s()`.

## 9. GPIO
Configure os GPIOs de saída como:
- Mode = `Output Push Pull`
- Pull = `No Pull`
- Speed = `Low`

Configure `SENSOR` como:
- Mode = `Input`
- Pull = `No Pull` (ou conforme sua eletrônica)

## 10. Gerar o projeto
1. Clique em **Generate Code**.
2. Copie os arquivos modulares deste pacote para `Core/Inc` e `Core/Src`.
3. Integre o `main.c` usando `main_example.c` como referência.
4. Aplique o patch do `USART1_IRQHandler()` conforme `stm32f3xx_it_patch_example.c`.

## 11. Integração no main.c
Dentro do `main.c` gerado pelo CubeMX:
- inclua `#include "mi_app.h"`
- depois das inicializações do CubeMX, chame `mi_app_init();`
- no loop infinito, chame `mi_app_process();`
- no `HAL_TIM_PeriodElapsedCallback()`, chame `mi_app_on_period_elapsed(htim);`

## 12. Observações importantes
- O **STM32F301C8T6** tem até **64 KB de Flash**, **16 KB de SRAM**, um timer avançado, até três timers gerais, um timer básico, até três USARTs, um ADC e um op-amp. O projeto sugerido usa principalmente `TIM1`, `TIM3`, `TIM6` e `USART1`.
- O **HSI interno é 8 MHz** e o dispositivo opera até **72 MHz**, o que é adequado para este MI modularizado.
- A família usa **página de Flash de 2 KB**, e o exemplo grava os parâmetros na última página de Flash (`0x0800F800`).
