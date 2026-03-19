#include "emulator.h"
#include "mcs51_emulator.h"

#include <string.h>
#include <stdio.h>
#include <cmsis_os2.h>

enum
{
  CODE_MEM_SIZE = 1024,
  XDATA_MEM_SIZE = 256,
  EMU_STEPS_PER_SLICE = 256,
  EMU_SLICE_DELAY_MS = 100,
};

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

  printf("Build at %s %s\n", __DATE__, __TIME__);
  printf("MCS51 emulator minimal mode\n");

  cpu = cpu_template;
  mem_map_attach(&cpu, &mem);
  memset(xdata_mem, 0x00, sizeof(xdata_mem));
  emulator_load_min_program();

  while (1)
  {
    uint64_t steps = cpu_run(&cpu, EMU_STEPS_PER_SLICE);
    if ((slice % 10u) == 0u)
    {
      printf("slice=%lu steps=%llu PC=0x%04X A=0x%02X SP=0x%02X\n",
             (unsigned long)slice,
             (unsigned long long)steps,
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

    slice++;
    osDelay(EMU_SLICE_DELAY_MS);
  }
}
