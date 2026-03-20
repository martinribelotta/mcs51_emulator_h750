#include "emulator.h"
#include "mcs51_emulator.h"
#include "tim.h"
#include "usart.h"

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
  UART_RX_RING_SIZE = 128,
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
static uart_t uart_dev;
static uint8_t usart1_rx_irq_byte;
static volatile uint16_t usart1_rx_head = 0;
static volatile uint16_t usart1_rx_tail = 0;
static uint8_t usart1_rx_ring[UART_RX_RING_SIZE];

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

static void uart_tick_hook(cpu_t *cpu_ctx, uint32_t cycles, void *user)
{
  (void)cpu_ctx;
  uart_t *uart = (uart_t *)user;
  uart_tick(uart, cycles);
}

static const cpu_tick_entry_t tick_hooks[] = {
  { uart_tick_hook, &uart_dev },
};

static void uart_tx_to_stm32(uint8_t byte, void *user)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)user;
  (void)HAL_UART_Transmit(huart, &byte, 1u, HAL_MAX_DELAY);
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

static void emulator_load_min_program(void)
{
  static const uint8_t demo_prog[] = {
      0x75, 0x89, 0x20,       /* MOV TMOD,#20h */
      0x75, 0x8D, 0xFD,       /* MOV TH1,#0FDh */
      0x75, 0x8B, 0xFD,       /* MOV TL1,#0FDh */
      0xD2, 0x8E,             /* SETB TR1 */
      0x75, 0x98, 0x50,       /* MOV SCON,#50h */
      0xD2, 0x99,             /* SETB TI */
      0x90, 0x00, 0x31,       /* MOV DPTR,#0031h */
      0xE4,                   /* CLR A */
      0x93,                   /* MOVC A,@A+DPTR */
      0x60, 0x06,             /* JZ echo_loop */
      0x12, 0x00, 0x29,       /* LCALL putc */
      0xA3,                   /* INC DPTR */
      0x80, 0xF6,             /* SJMP print_loop */
      0x30, 0x98, 0xFD,       /* echo_loop: JNB RI,echo_loop */
      0xE5, 0x99,             /* MOV A,SBUF */
      0xC2, 0x98,             /* CLR RI */
      0x12, 0x00, 0x29,       /* LCALL putc */
      0x80, 0xF4,             /* SJMP echo_loop */
      0x30, 0x99, 0xFD,       /* putc: JNB TI,putc */
      0xC2, 0x99,             /* CLR TI */
      0xF5, 0x99,             /* MOV SBUF,A */
      0x22,                   /* RET */
      'M','C','S','5','1',' ','U','A','R','T','1',' ','d','e','m','o','\r','\n',
      'E','c','h','o',':',' ','\r','\n',
      0x00,
  };

  memset(code_mem, 0x00, sizeof(code_mem));
  (void)memcpy(code_mem, demo_prog, sizeof(demo_prog));
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
  uart_init(&uart_dev, &timing_cfg);
  uart_attach(&cpu, &uart_dev);
  uart_set_tx_callback(&uart_dev, uart_tx_to_stm32, &huart1);
  cpu_set_tick_hooks(&cpu, tick_hooks, (uint8_t)MCS51_ARRAY_LEN(tick_hooks));
  usart1_rx_start_it();

  memset(xdata_mem, 0x00, sizeof(xdata_mem));
  emulator_load_min_program();

  while (1)
  {
    uart_rx_from_stm32_irq_drain();

    timing_reset(&timing_state);
    uint64_t steps = cpu_run_timed(&cpu,
                                   EMU_STEPS_PER_SLICE,
                                   &timing_cfg,
                                   &timing_state,
                                   &time_iface);

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
  }
}
