#ifndef SLIME_GBA_H
#define SLIME_GBA_H

#include <stdint.h>
#include "../arm/arm.h"
#include "../memory/memory.h"

typedef struct Gba {

  Arm arm;
  Memory memory;

} Gba ;

void power_on_gba(uint8_t *rom, unsigned int rom_size);

#endif
