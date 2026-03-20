# h750-test1

STM32H750 + FreeRTOS firmware that runs an embedded **MCS-51 (8051) emulator** in real time, bridges emulated UART traffic to hardware `USART1`, and exposes virtual 8051 GPIO ports (`P0..P3`) with a hardware mapping from `P1.0` to `PA8` (LED).

## What this project does

- Boots on STM32H750 using STM32Cube HAL + FreeRTOS.
- Starts an emulator task (`emulator_entry`) from the default FreeRTOS thread.
- Runs a software MCS-51 CPU core with:
  - 64 KB code memory
  - 64 KB XDATA memory
  - UART peripheral emulation
  - Port peripheral emulation (`P0..P3`)
- Uses `TIM5` as a high-resolution time base for paced emulation.
- Bridges emulated serial I/O to physical `USART1`.
- Loads a built-in 8051 demo program that:
  - prints a startup message over serial,
  - then echoes received bytes,
  - toggles `P1.0` on each echoed character.
- Reflects `P1.0` to STM32 `PA8`, so the LED toggles while echoing.

## Current implementation summary

### Runtime architecture

1. MCU init (`main.c`) initializes:
   - GPIO
   - USART1
   - TIM5
   - FreeRTOS kernel
2. FreeRTOS starts `StartDefaultTask`.
3. `StartDefaultTask` calls `emulator_entry()`.
4. Emulator sets up CPU, memory map, UART, ports, time interface, and starts main emulation loop.

### Emulation timing model

- Target 8051 oscillator is configured to **11.0592 MHz**.
- `clocks_per_cycle = 12`, so machine-cycle pacing is based on:
  - `11059200 / 12 = 921600 cycles/s`.
- `cpu_run_timed()` uses a custom time interface backed by `TIM5` counter ticks.

### UART bridge (MCS-51 <-> USART1)

- **TX path:** emulated UART TX callback writes bytes through `HAL_UART_Transmit()` on `USART1`.
- **RX path:** interrupt-driven on STM32 side:
  - `USART1_IRQHandler` -> HAL IRQ handler
  - `HAL_UART_RxCpltCallback` pushes bytes into a small ring buffer
  - emulator loop drains ring buffer and feeds bytes into emulated UART RX queue (`uart_queue_rx_byte`)

### Virtual GPIO ports

- MCS-51 ports `P0..P3` are emulated with the emulator `ports` peripheral.
- External readback uses virtual input arrays.
- Port writes update virtual output arrays.
- Mapping implemented:
  - `P1.0` -> STM32 `PA8` output (LED)

## Pin mapping

- `USART1_TX`: `PA9`
- `USART1_RX`: `PA10`
- `P1.0` (emulated): mapped to `PA8` (LED output)

## Build

This project uses CMake presets and Ninja.

### Configure + Build (Debug)

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Generated ELF (Debug):

- `build/Debug/h750-test1.elf`

## Flash / Debug

Use your usual STM32 workflow (STM32CubeIDE, OpenOCD, ST-LINK, or VS Code launch config) to flash and run `h750-test1.elf`.

## How to test quickly

1. Flash firmware and start target.
2. Open serial terminal on `USART1` (8N1).
3. Observe startup text from emulated 8051.
4. Type characters:
   - They should be echoed back.
   - `PA8` LED should toggle on each echoed byte.

## Project structure (relevant parts)

- `Core/Src/main.c` - board/system init and scheduler start.
- `Core/Src/freertos.c` - default task that starts emulator.
- `Core/Src/usart.c` - USART1 init and IRQ enable.
- `Core/Src/stm32h7xx_it.c` - `USART1_IRQHandler`.
- `Core/Src/gpio.c` - `PA8` LED output init.
- `Middlewares/emulator/emulator.c` - emulator integration glue (timing, UART bridge, virtual GPIO, demo firmware).
- `Middlewares/emulator/mcs51_emulator/lib` - MCS-51 core/peripherals library.

## Notes and limitations

- The current demo firmware is embedded as raw opcodes in `emulator.c`.
- UART RX uses interrupt + ring buffer (simple, lightweight design).
- Emulation pacing is best-effort real time on top of FreeRTOS + HAL.
