# MI modularizado para STM32F301C8T6

Este pacote reorganiza a lógica do MI em módulos para manter o `main.c` gerado pelo CubeMX o mais limpo possível.

## Estrutura
- `Core/Inc/mi_types.h`: tipos, enums e defines comuns
- `Core/Inc/mi_state.h` / `Core/Src/mi_state.c`: estado global compartilhado
- `Core/Inc/mi_comm.h` / `Core/Src/mi_comm.c`: protocolo RS485, parser e processamento de frames
- `Core/Inc/mi_motor.h` / `Core/Src/mi_motor.c`: SPWM, rampa, telemetria e persistência
- `Core/Inc/mi_peripherals.h` / `Core/Src/mi_peripherals.c`: sensor, bomba, swing, dreno e lógica remota
- `Core/Inc/mi_app.h` / `Core/Src/mi_app.c`: cola entre os módulos e o `main.c`
- `Core/Inc/mi_rs485_dma_shared.h`: variáveis compartilhadas entre `main/mi_comm` e `stm32f3xx_it.c`
- `Core/Src/main_example.c`: exemplo mínimo de `main.c`
- `Core/Src/stm32f3xx_it_patch_example.c`: exemplo do patch de recepção DMA + IDLE

## Observações
- Este pacote foi estruturado para integrar com um projeto gerado pelo STM32CubeMX/STM32CubeIDE para **STM32F301C8T6**.
- Os arquivos gerados pelo CubeMX (`main.c`, `tim.c`, `usart.c`, `gpio.c`, `dma.c`, `stm32f3xx_it.c`) continuam sendo a base do projeto.
- Os nomes dos pinos (`BOMBA_Pin`, `RS485_EN_Pin`, etc.) dependem do `main.h` gerado pelo CubeMX.
- A lógica de telemetria permanece compatível com o MI unificado usado anteriormente.
