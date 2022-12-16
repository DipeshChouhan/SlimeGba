#include "gba.h"
#include "../arm/arm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../arm/disassembler.h"

int processor_modes[16] = {USR, FIQ, IRQ, SVC, 7, 7, 7, ABT,
                           7,   7,   7,   UND, 7, 7, 7, SYS};

void load_gba_rom(Gba *gba) {

}

void power_on_gba(uint8_t *rom, unsigned int rom_size) {
  // printf("file open of size - %d\n", rom_size);
  Gba gba;
}
