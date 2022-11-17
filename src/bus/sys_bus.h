#ifndef SLIME_SYS_BUS_H 
#define SLIME_SYS_BUS_H
#include <stdint.h>

typedef struct SysBus {

  uint8_t bios_rom[16 * 1024];
  uint8_t wram[256 * 1024]; // on board work ram
  uint8_t io[1024]; // io registers
  uint8_t palette[1024]; 
  uint8_t vram[96 * 1024];
  uint8_t oam[1024];
} SysBus ;


#endif
