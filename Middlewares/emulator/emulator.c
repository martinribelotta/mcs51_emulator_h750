#include "emulator.h"

#include <stdio.h>
#include <cmsis_os2.h>

void emulator_entry(void)
{
  int counter = 0;

  printf("Build at %s %s\n", __DATE__, __TIME__);
  while (1)
  {
    osDelay(250);
    printf("Counter: %d\n", counter);
    counter++;
  }
}
