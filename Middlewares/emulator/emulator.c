#include "emulator.h"
#include "mcs51_emulator.h"
#include "tim.h"

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <cmsis_os2.h>

enum
{
  CODE_MEM_SIZE = 64 * 1024,
  XDATA_MEM_SIZE = 64 * 1024,
  EMU_STEPS_PER_SLICE = 256,
  EMU_TARGET_FOSC_HZ = 11059200,
  EMU_CLOCKS_PER_CYCLE = 12,
  NS_PER_SEC = 1000000000,
};

static bool tim5_started = false;
static uint64_t tim5_counter_hz = 0;

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
  (void)user;
  if (ns == 0ull)
  {
    osThreadYield();
    return;
  }

  if (!tim5_started || tim5_counter_hz == 0ull)
  {
    osThreadYield();
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
    return;
  }

  uint64_t end_tick = start_tick + wait_ticks;
  while ((int32_t)((uint32_t)__HAL_TIM_GET_COUNTER(&htim5) - (uint32_t)end_tick) < 0)
  {
    __NOP();
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

static void emulator_load_min_program(void)
{
  memset(code_mem, 0x00, sizeof(code_mem));

  code_mem[0] = 0x74; /* MOV A,#imm */
  code_mem[1] = 0x2A;
  code_mem[2] = 0x04; /* INC A */
  code_mem[3] = 0x80; /* SJMP rel */
  code_mem[4] = 0xFD; /* back to 0x0002 */
}

void emulator_entry(void)
{
  uint32_t slice = 0;
  uint32_t cycles_per_second = (uint32_t)timing_cycles_per_second(&timing_cfg);
  uint32_t ns_per_cycle = (uint32_t)timing_cycles_to_ns(&timing_cfg, 1u);

  printf("Build at %s %s\n", __DATE__, __TIME__);
  printf("MCS51 emulator minimal mode\n");
    printf("Timing: fosc=%lu Hz clocks/cycle=%u cycles/s=%lu ns/cycle=%lu\n",
         (unsigned long)timing_cfg.fosc_hz,
         (unsigned int)timing_cfg.clocks_per_cycle,
      (unsigned long)cycles_per_second,
      (unsigned long)ns_per_cycle);

  tim5_timebase_init();
  if (tim5_counter_hz == 0ull)
  {
    printf("TIM5 timebase invalid\n");
    return;
  }
    printf("TIM5 counter=%lu Hz tick_ns=%lu\n",
      (unsigned long)tim5_counter_hz,
      (unsigned long)((uint64_t)NS_PER_SEC / tim5_counter_hz));

  cpu = cpu_template;
  mem_map_attach(&cpu, &mem);
  memset(xdata_mem, 0x00, sizeof(xdata_mem));
  emulator_load_min_program();

  while (1)
  {
    timing_reset(&timing_state);
    uint64_t steps = cpu_run_timed(&cpu,
                                   EMU_STEPS_PER_SLICE,
                                   &timing_cfg,
                                   &timing_state,
                                   &time_iface);
    uint32_t steps_u32 = (uint32_t)steps;
    uint32_t emu_us_u32 = (uint32_t)timing_cycles_to_us(&timing_cfg, timing_state.cycles_total);
    if ((slice % 10u) == 0u)
    {
      printf("slice=%lu steps=%lu emu_us=%lu PC=0x%04X A=0x%02X SP=0x%02X\n",
             (unsigned long)slice,
             (unsigned long)steps_u32,
             (unsigned long)emu_us_u32,
             (unsigned int)cpu.pc,
             (unsigned int)cpu.acc,
             (unsigned int)cpu.sp);
    }

    if (cpu.halted)
    {
      printf("CPU halted PC=0x%04X opcode=0x%02X reason=%s\n",
             (unsigned int)cpu.pc,
             (unsigned int)cpu.last_opcode,
             cpu.halt_reason ? cpu.halt_reason : "unknown");
      break;
    }

    if (steps == 0u)
    {
      osThreadYield();
    }

    slice++;
  }
}
