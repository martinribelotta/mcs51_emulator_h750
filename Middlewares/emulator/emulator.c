#include "emulator.h"
#include "mcs51_emulator.h"
#include "tim.h"
#include "uart.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <stdint.h>
#include <cmsis_os2.h>

enum
{
  CODE_MEM_SIZE = 64 * 1024,
  XDATA_MEM_SIZE = 64 * 1024,
  EMU_STEPS_PER_SLICE = 1,
  EMU_TARGET_FOSC_HZ = 11059200,
  EMU_CLOCKS_PER_CYCLE = 12,
  NS_PER_SEC = 1000000000,
  UART_RX_RING_SIZE = 128,
  DWT_REPORT_EVERY_SAMPLES = 50000,
};

static bool tim5_started = false;
static uint64_t tim5_counter_hz = 0;

static bool dwt_sleep_prof_enabled(void);
static void dwt_sleep_profiler_record(uint32_t delta_cycles);

static uint64_t tim5_get_input_clock_hz(void)
{
  RCC_ClkInitTypeDef clk = {0};
  uint32_t flash_latency = 0;
  HAL_RCC_GetClockConfig(&clk, &flash_latency);

  uint64_t pclk1_hz = (uint64_t)HAL_RCC_GetPCLK1Freq();
  if (clk.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    return pclk1_hz;
  }
  return pclk1_hz * 2ull;
}

static void tim5_timebase_init(void)
{
  if (tim5_started)
  {
    return;
  }

  uint64_t tim_in_hz = tim5_get_input_clock_hz();
  uint32_t psc = htim5.Instance->PSC;
  uint64_t div = (uint64_t)psc + 1ull;
  if (div == 0ull)
  {
    div = 1ull;
  }
  tim5_counter_hz = tim_in_hz / div;

  __HAL_TIM_SET_COUNTER(&htim5, 0u);
  (void)HAL_TIM_Base_Start(&htim5);
  tim5_started = true;
}

static uint64_t freertos_now_ns(void *user)
{
  (void)user;

  if (!tim5_started || tim5_counter_hz == 0ull)
  {
    return 0ull;
  }

  uint64_t ticks = (uint64_t)__HAL_TIM_GET_COUNTER(&htim5);
  return (ticks * (uint64_t)NS_PER_SEC) / tim5_counter_hz;
}

static void freertos_sleep_ns(uint64_t ns, void *user)
{
  uint32_t dwt_start = 0u;
  if (dwt_sleep_prof_enabled())
  {
    dwt_start = DWT->CYCCNT;
  }

  (void)user;
  if (ns == 0ull)
  {
    osThreadYield();
    if (dwt_sleep_prof_enabled())
    {
      dwt_sleep_profiler_record(DWT->CYCCNT - dwt_start);
    }
    return;
  }

  if (!tim5_started || tim5_counter_hz == 0ull)
  {
    osThreadYield();
    if (dwt_sleep_prof_enabled())
    {
      dwt_sleep_profiler_record(DWT->CYCCNT - dwt_start);
    }
    return;
  }

  if (ns >= 2000000ull)
  {
    uint32_t delay_ms = (uint32_t)(ns / 1000000ull);
    if (delay_ms > 0u)
    {
      osDelay(delay_ms);
    }
  }

  uint64_t start_tick = (uint64_t)__HAL_TIM_GET_COUNTER(&htim5);
  uint64_t wait_ticks = (ns * tim5_counter_hz + ((uint64_t)NS_PER_SEC - 1ull)) / (uint64_t)NS_PER_SEC;
  if (wait_ticks == 0ull)
  {
    __NOP();
    if (dwt_sleep_prof_enabled())
    {
      dwt_sleep_profiler_record(DWT->CYCCNT - dwt_start);
    }
    return;
  }

  uint64_t end_tick = start_tick + wait_ticks;
  while ((int32_t)((uint32_t)__HAL_TIM_GET_COUNTER(&htim5) - (uint32_t)end_tick) < 0)
  {
    __NOP();
  }

  if (dwt_sleep_prof_enabled())
  {
    dwt_sleep_profiler_record(DWT->CYCCNT - dwt_start);
  }
}

static uint8_t ram_read(const cpu_t *cpu, uint16_t addr, void *user)
{
  (void)cpu;
  uint8_t *ram = (uint8_t *)user;
  return ram[addr];
}

static void ram_write(cpu_t *cpu, uint16_t addr, uint8_t value, void *user)
{
  (void)cpu;
  uint8_t *ram = (uint8_t *)user;
  ram[addr] = value;
}

static const cpu_t cpu_template = CPU_INIT_TEMPLATE_INIT;
static cpu_t cpu;
static uint8_t code_mem[CODE_MEM_SIZE];
static uint8_t xdata_mem[XDATA_MEM_SIZE];
static timers_t timers_dev;
static uart_t uart_dev;
static ports_t ports_dev;
static uint8_t usart1_rx_irq_byte;
static volatile uint16_t usart1_rx_head = 0;
static volatile uint16_t usart1_rx_tail = 0;
static uint8_t usart1_rx_ring[UART_RX_RING_SIZE];
static uint8_t virtual_port_in[4] = { 0xFFu, 0xFFu, 0xFFu, 0xFFu };
static uint8_t virtual_port_out[4] = { 0x00u, 0x00u, 0x00u, 0x00u };

static const timing_config_t timing_cfg = {
  .fosc_hz = EMU_TARGET_FOSC_HZ,
  .clocks_per_cycle = EMU_CLOCKS_PER_CYCLE,
};
static timing_state_t timing_state = {
  .cycles_total = 0,
};
static const cpu_time_iface_t time_iface = {
  .now_ns = freertos_now_ns,
  .sleep_ns = freertos_sleep_ns,
  .user = NULL,
};

typedef struct
{
  bool enabled;
  bool report_ready;
  uint32_t min_total;
  uint32_t max_total;
  uint32_t samples_since_report;
  uint64_t total_step;
  uint64_t total_hook;
  uint64_t total_timing;
  uint64_t total_total;
  uint64_t sample_count;
  uint32_t report_step_avg;
  uint32_t report_hook_avg;
  uint32_t report_timing_avg;
  uint32_t report_total_avg;
  uint32_t report_total_min;
  uint32_t report_total_max;
  uint32_t report_samples;
} dwt_tick_profiler_t;

static dwt_tick_profiler_t dwt_prof = {
  .enabled = false,
  .report_ready = false,
  .min_total = UINT32_MAX,
  .max_total = 0u,
  .samples_since_report = 0u,
  .total_step = 0ull,
  .total_hook = 0ull,
  .total_timing = 0ull,
  .total_total = 0ull,
  .sample_count = 0ull,
  .report_step_avg = 0u,
  .report_hook_avg = 0u,
  .report_timing_avg = 0u,
  .report_total_avg = 0u,
  .report_total_min = 0u,
  .report_total_max = 0u,
  .report_samples = 0u,
};

typedef struct
{
  bool enabled;
  uint32_t min_cycles;
  uint32_t max_cycles;
  uint64_t total_cycles;
  uint64_t call_count;
  uint32_t report_min;
  uint32_t report_max;
  uint32_t report_avg;
  uint32_t report_calls;
} dwt_sleep_profiler_t;

static dwt_sleep_profiler_t dwt_sleep_prof = {
  .enabled = false,
  .min_cycles = UINT32_MAX,
  .max_cycles = 0u,
  .total_cycles = 0ull,
  .call_count = 0ull,
  .report_min = 0u,
  .report_max = 0u,
  .report_avg = 0u,
  .report_calls = 0u,
};

static bool dwt_sleep_prof_enabled(void)
{
  return dwt_sleep_prof.enabled;
}

static void dwt_sleep_profiler_record(uint32_t delta_cycles)
{
  if (!dwt_sleep_prof.enabled)
  {
    return;
  }

  if (delta_cycles < dwt_sleep_prof.min_cycles)
  {
    dwt_sleep_prof.min_cycles = delta_cycles;
  }
  if (delta_cycles > dwt_sleep_prof.max_cycles)
  {
    dwt_sleep_prof.max_cycles = delta_cycles;
  }
  dwt_sleep_prof.total_cycles += (uint64_t)delta_cycles;
  dwt_sleep_prof.call_count++;
}

static uint32_t dwt_cycle_read(void *user)
{
  (void)user;
  return DWT->CYCCNT;
}

static void dwt_run_timed_sample(uint32_t step_cycles,
                                 uint32_t hook_cycles,
                                 uint32_t timing_cycles,
                                 uint32_t total_cycles,
                                 uint8_t emu_cycles,
                                 void *user)
{
  (void)emu_cycles;
  (void)user;

  if (!dwt_prof.enabled)
  {
    return;
  }

  dwt_prof.total_step += (uint64_t)step_cycles;
  dwt_prof.total_hook += (uint64_t)hook_cycles;
  dwt_prof.total_timing += (uint64_t)timing_cycles;
  dwt_prof.total_total += (uint64_t)total_cycles;

  if (total_cycles < dwt_prof.min_total)
  {
    dwt_prof.min_total = total_cycles;
  }
  if (total_cycles > dwt_prof.max_total)
  {
    dwt_prof.max_total = total_cycles;
  }

  dwt_prof.sample_count++;
  dwt_prof.samples_since_report++;

  if (dwt_prof.samples_since_report >= DWT_REPORT_EVERY_SAMPLES && dwt_prof.sample_count > 0u)
  {
    dwt_prof.report_step_avg = (uint32_t)(dwt_prof.total_step / dwt_prof.sample_count);
    dwt_prof.report_hook_avg = (uint32_t)(dwt_prof.total_hook / dwt_prof.sample_count);
    dwt_prof.report_timing_avg = (uint32_t)(dwt_prof.total_timing / dwt_prof.sample_count);
    dwt_prof.report_total_avg = (uint32_t)(dwt_prof.total_total / dwt_prof.sample_count);
    dwt_prof.report_total_min = dwt_prof.min_total;
    dwt_prof.report_total_max = dwt_prof.max_total;
    dwt_prof.report_samples = (uint32_t)dwt_prof.sample_count;
    dwt_prof.report_ready = true;

    dwt_prof.min_total = UINT32_MAX;
    dwt_prof.max_total = 0u;
    dwt_prof.samples_since_report = 0u;
    dwt_prof.total_step = 0ull;
    dwt_prof.total_hook = 0ull;
    dwt_prof.total_timing = 0ull;
    dwt_prof.total_total = 0ull;
    dwt_prof.sample_count = 0ull;
  }
}

static const cpu_run_timed_profiler_t run_timed_profiler = {
  .read_cycles = dwt_cycle_read,
  .on_sample = dwt_run_timed_sample,
  .user = NULL,
};

static void dwt_profiler_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  dwt_prof.enabled = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
  dwt_sleep_prof.enabled = dwt_prof.enabled;
  dwt_prof.report_ready = false;
  dwt_prof.min_total = UINT32_MAX;
  dwt_prof.max_total = 0u;
  dwt_prof.samples_since_report = 0u;
  dwt_prof.total_step = 0ull;
  dwt_prof.total_hook = 0ull;
  dwt_prof.total_timing = 0ull;
  dwt_prof.total_total = 0ull;
  dwt_prof.sample_count = 0ull;

  dwt_sleep_prof.min_cycles = UINT32_MAX;
  dwt_sleep_prof.max_cycles = 0u;
  dwt_sleep_prof.total_cycles = 0ull;
  dwt_sleep_prof.call_count = 0ull;
  dwt_sleep_prof.report_min = 0u;
  dwt_sleep_prof.report_max = 0u;
  dwt_sleep_prof.report_avg = 0u;
  dwt_sleep_prof.report_calls = 0u;
}

static void dwt_profiler_maybe_report(void)
{
  if (!dwt_prof.report_ready)
  {
    return;
  }

  dwt_prof.report_ready = false;

  if (dwt_sleep_prof.call_count > 0u)
  {
    dwt_sleep_prof.report_calls = (uint32_t)dwt_sleep_prof.call_count;
    dwt_sleep_prof.report_min = dwt_sleep_prof.min_cycles;
    dwt_sleep_prof.report_max = dwt_sleep_prof.max_cycles;
    dwt_sleep_prof.report_avg = (uint32_t)(dwt_sleep_prof.total_cycles / dwt_sleep_prof.call_count);

    dwt_sleep_prof.min_cycles = UINT32_MAX;
    dwt_sleep_prof.max_cycles = 0u;
    dwt_sleep_prof.total_cycles = 0ull;
    dwt_sleep_prof.call_count = 0ull;
  }

    printf("DWT run_timed cycles: step=%lu hooks=%lu timing=%lu total=%lu min=%lu max=%lu samples=%lu\n",
      (unsigned long)dwt_prof.report_step_avg,
      (unsigned long)dwt_prof.report_hook_avg,
      (unsigned long)dwt_prof.report_timing_avg,
      (unsigned long)dwt_prof.report_total_avg,
      (unsigned long)dwt_prof.report_total_min,
      (unsigned long)dwt_prof.report_total_max,
         (unsigned long)dwt_prof.report_samples);

  if (dwt_sleep_prof.report_calls > 0u)
  {
    printf("DWT freertos_sleep_ns cycles: avg=%lu min=%lu max=%lu calls=%lu\n",
           (unsigned long)dwt_sleep_prof.report_avg,
           (unsigned long)dwt_sleep_prof.report_min,
           (unsigned long)dwt_sleep_prof.report_max,
           (unsigned long)dwt_sleep_prof.report_calls);
  }
}

static void emulator_tick_hook(cpu_t *cpu_ctx, uint32_t cycles, void *user)
{
  (void)cpu_ctx;
  (void)user;
  timers_tick(&timers_dev, cycles);
  uart_tick(&uart_dev, cycles);
}

static const cpu_tick_entry_t tick_hooks[] = {
  { emulator_tick_hook, NULL },
};

static void uart_change_baud_stdout(uint32_t baud, void *user)
{
    FILE *out = user ? (FILE *)user : stdout;
    fprintf(out, "UART baud rate changed: %lu\n", baud);
}

static void uart_tx_to_stm32(uint8_t byte, void *user)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)user;
  (void)HAL_UART_Transmit(huart, &byte, 1u, HAL_MAX_DELAY);
}

static uint8_t ports_read_virtual(uint8_t port, void *user)
{
  (void)user;
  if (port >= 4u)
  {
    return 0xFFu;
  }
  return virtual_port_in[port];
}

static void ports_write_virtual(uint8_t port, uint8_t level, uint8_t mask, void *user)
{
  (void)user;
  if (port >= 4u)
  {
    return;
  }

  virtual_port_out[port] = (uint8_t)((virtual_port_out[port] & (uint8_t)~mask) | (level & mask));

  if (port == 1u && (mask & 0x01u) != 0u)
  {
    GPIO_PinState led_state = ((level & 0x01u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, led_state);
  }
}

static void usart1_rx_start_it(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_irq_byte, 1u);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t head = usart1_rx_head;
    uint16_t next = (uint16_t)((head + 1u) % UART_RX_RING_SIZE);
    if (next != usart1_rx_tail)
    {
      usart1_rx_ring[head] = usart1_rx_irq_byte;
      usart1_rx_head = next;
    }
    usart1_rx_start_it();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    usart1_rx_start_it();
  }
}

static void uart_rx_from_stm32_irq_drain(void)
{
  while (1)
  {
    uint16_t tail;
    uint8_t byte;

    __disable_irq();
    if (usart1_rx_tail == usart1_rx_head)
    {
      __enable_irq();
      break;
    }
    tail = usart1_rx_tail;
    byte = usart1_rx_ring[tail];
    __enable_irq();

    if (!uart_queue_rx_byte(&uart_dev, byte))
    {
      break;
    }

    __disable_irq();
    usart1_rx_tail = (uint16_t)((tail + 1u) % UART_RX_RING_SIZE);
    __enable_irq();
  }
}

static const mem_map_region_t code_regions[] = {
  { .base = 0x0000, .size = CODE_MEM_SIZE, .read = ram_read, .write = ram_write, .user = code_mem },
};

static const mem_map_region_t xdata_regions[] = {
  { .base = 0x0000, .size = XDATA_MEM_SIZE, .read = ram_read, .write = ram_write, .user = xdata_mem },
};

static const mem_map_t mem = {
    .code_regions = code_regions,
    .code_region_count = 1,
    .xdata_regions = xdata_regions,
    .xdata_region_count = 1,
};

extern const uint8_t mcs51_firmware_start[];
extern const uint8_t mcs51_firmware_end[];

static void emulator_load_embedded_firmware(void)
{
  uintptr_t start = (uintptr_t)mcs51_firmware_start;
  uintptr_t end = (uintptr_t)mcs51_firmware_end;
  size_t fw_size = (size_t)(end - start);
  memset(code_mem, 0x00, sizeof(code_mem));
  (void)memcpy(code_mem, mcs51_firmware_start, fw_size);
  printf("Embedded firmware loaded: %lu bytes\n", (unsigned long)fw_size);
}

void trace_stdout(cpu_t *cpu, uint16_t pc, uint8_t opcode, const char *name, void *user)
{
    FILE *out = user ? (FILE *)user : stdout;
    fprintf(out,
            "%04X  %02X  %-16s  A=%02X PSW=%02X SP=%02X DPTR=%04X\n",
            pc,
            opcode,
            name,
            cpu->acc,
            cpu->psw,
            cpu->sp,
            cpu->dptr);
}

void emulator_entry(void)
{
  uint32_t cycles_per_second = (uint32_t)timing_cycles_per_second(&timing_cfg);
  uint32_t ns_per_cycle = (uint32_t)timing_cycles_to_ns(&timing_cfg, 1u);

  printf("Build at %s %s\n", __DATE__, __TIME__);
  printf("MCS51 emulator minimal mode\n");
  printf("Timing: fosc=%lu Hz clocks/cycle=%u cycles/s=%lu ns/cycle=%lu\n",
        (unsigned long)timing_cfg.fosc_hz,
        (unsigned int)timing_cfg.clocks_per_cycle,
        (unsigned long)cycles_per_second,
        (unsigned long)ns_per_cycle);
  printf("MCS51 UART demo bridged to USART1\n");
  printf("Virtual GPIO: P0..P3 enabled, P1.0 -> PA8 LED\n");

  tim5_timebase_init();
  dwt_profiler_init();
  if (dwt_prof.enabled)
  {
    cpu_set_run_timed_profiler(&run_timed_profiler);
  }
  else
  {
    cpu_set_run_timed_profiler(NULL);
  }
  if (tim5_counter_hz == 0ull)
  {
    printf("TIM5 timebase invalid\n");
    return;
  }
  printf("TIM5 counter=%lu Hz tick_ns=%lu\n",
    (unsigned long)tim5_counter_hz,
    (unsigned long)((uint64_t)NS_PER_SEC / tim5_counter_hz));
  printf("DWT profiler: %s\n", dwt_prof.enabled ? "enabled" : "unavailable");

  cpu = cpu_template;
  mem_map_attach(&cpu, &mem);
  timers_init(&timers_dev, &cpu);
  timers_set_profile(&timers_dev, TIMERS_PROFILE_PRAGMATIC);
  ports_init(&ports_dev, &cpu, ports_read_virtual, ports_write_virtual, NULL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  uart_init(&uart_dev, &timing_cfg);
  uart_attach(&cpu, &uart_dev);
  uart_set_baud_callback(&uart_dev, uart_change_baud_stdout, stdout);
  uart_set_tx_callback(&uart_dev, uart_tx_to_stm32, &huart1);
  cpu_set_tick_hooks(&cpu, tick_hooks, (uint8_t)MCS51_ARRAY_LEN(tick_hooks));
  // cpu_set_trace(&cpu, true, trace_stdout, stdout);
  usart1_rx_start_it();

  memset(xdata_mem, 0x00, sizeof(xdata_mem));
  emulator_load_embedded_firmware();
  while (1)
  {
    uart_rx_from_stm32_irq_drain();

    timing_reset(&timing_state);
    cpu_run_timed(&cpu,
                  EMU_STEPS_PER_SLICE,
                  &timing_cfg,
                  &timing_state,
                  NULL);

    dwt_profiler_maybe_report();

    if (cpu.halted)
    {
      printf("CPU halted PC=0x%04X opcode=0x%02X reason=%s\n",
             (unsigned int)cpu.pc,
             (unsigned int)cpu.last_opcode,
             cpu.halt_reason ? cpu.halt_reason : "unknown");
      break;
    }
  }
}
