#include "gba.h"
#include "../arm/arm.h"
#include <stdio.h>


void power_on_gba(uint8_t *rom, unsigned int rom_size) {
  // printf("file open of size - %d\n", rom_size);
  Arm arm;
  init_arm(&arm);
}
