#include "gba.h"
#include "../arm/arm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void power_on_gba(uint8_t *rom, unsigned int rom_size) {
  // printf("file open of size - %d\n", rom_size);
  Gba gba;
  gba.arm.curr_instruction = 0x03000000;
  init_arm(&gba.arm);

  memcpy(&gba.memory.iwram, rom, rom_size);
  free(rom);
  int index = 0;
  int total = 0;
  while (index < rom_size) {
    thumb_exec(&gba.arm);
    index += 2;
    ++total;
  }
  printf("called %d\n", total);
}
