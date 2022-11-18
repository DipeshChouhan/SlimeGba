#ifndef SLIME_MEMORY_H 
#define SLIME_MEMORY_H
#include <stdint.h>
#include "../arm/arm.h"


typedef struct Memory {

  uint8_t bios_rom[16 * 1024];
  uint8_t wram[256 * 1024]; // on board work ram
  uint8_t io[1024];         // io registers
  uint8_t palette[1024];
  uint8_t vram[96 * 1024];
  uint8_t oam[1024];
} Memory;

uint32_t mem_read32 (Arm *arm);
uint16_t mem_read16 ();
uint8_t mem_read8 ();

void mem_write32();
void mem_write16();
void mem_write8();

#endif
