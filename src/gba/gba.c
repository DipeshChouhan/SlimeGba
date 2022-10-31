#include "gba.h"
#include "../arm/arm.h"
#include <stdio.h>

void power_on_gba(uint8_t *rom, unsigned int rom_size) {
  printf("file open of size - %d\n", rom_size);
  Arm arm;
  arm.data_bus = 0xE8BD9FFF;

  arm_exec(&arm);
}
