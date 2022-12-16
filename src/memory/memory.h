#ifndef SLIME_MEMORY_H 
#define SLIME_MEMORY_H
#include <stdint.h>



typedef struct Memory {

  uint32_t address_bus;
  uint32_t data_bus;

  uint8_t bios_rom[16 * 1024];
  uint8_t iwram[32 * 1024]; // internal work ram
  uint8_t wram[256 * 1024]; // on board work ram
  uint8_t io_ram[1024];         // io registers
  uint8_t *game_pak_rom;
  uint8_t *sram;
  uint8_t palette_ram[1024];
  uint8_t vram[96 * 1024];
  uint8_t oam[1024];
  uint8_t *mem_table[13];
  uint32_t mem_mirrors[13];
} Memory;

void memory_init(Memory *mem);

uint32_t mem_read32 (Memory *mem);
uint32_t mem_read16 (Memory *mem);
uint32_t mem_read8 (Memory *mem);

void mem_write32(Memory *mem);
void mem_write16(Memory *mem);
void mem_write8(Memory *mem);

#endif
