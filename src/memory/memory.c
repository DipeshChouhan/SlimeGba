#include "memory.h"
#include <stdio.h>
// TODO correct memory accessing implementation

/*General Internal Memory
  00000000-00003FFF   BIOS - System ROM         (16 KBytes)
  00004000-01FFFFFF   Not used
  02000000-0203FFFF   WRAM - On-board Work RAM  (256 KBytes) 2 Wait
  02040000-02FFFFFF   Not used
  03000000-03007FFF   WRAM - On-chip Work RAM   (32 KBytes)
  03008000-03FFFFFF   Not used
  04000000-040003FE   I/O Registers
  04000400-04FFFFFF   Not used
Internal Display Memory 05000000-050003FF   BG/OBJ Palette RAM        (1 Kbyte) 05000400-05FFFFFF   Not used
  06000000-06017FFF   VRAM - Video RAM          (96 KBytes)
  06018000-06FFFFFF   Not used
  07000000-070003FF   OAM - OBJ Attributes      (1 Kbyte)
  07000400-07FFFFFF   Not used
External Memory (Game Pak)
  08000000-09FFFFFF   Game Pak ROM/FlashROM (max 32MB) - Wait State 0
  0A000000-0BFFFFFF   Game Pak ROM/FlashROM (max 32MB) - Wait State 1
  0C000000-0DFFFFFF   Game Pak ROM/FlashROM (max 32MB) - Wait State 2
  0E000000-0E00FFFF   Game Pak SRAM    (max 64 KBytes) - 8bit Bus width
  0E010000-0FFFFFFF   Not used
Unused Memory Area
  10000000-FFFFFFFF   Not used (upper 4bits of mem->address_bus bus unused)
  */

#define MEM_WRITE32(_ram)                                                      \
  _ram[mem->address_bus] = mem->data_bus;                                      \
  _ram[mem->address_bus + 1] = mem->data_bus >> 8;                             \
  _ram[mem->address_bus + 2] = mem->data_bus >> 16;                            \
  _ram[mem->address_bus + 3] = mem->data_bus >> 24;

#define MEM_WRITE16(_ram)                                                      \
  _ram[mem->address_bus] = mem->data_bus;                                      \
  _ram[mem->address_bus + 1] = mem->data_bus >> 8;

#define MEM_WRITE8(_ram) _ram[mem->address_bus] = mem->data_bus;

#define MEM_READ32(_ram)                                                       \
  mem->data_bus = (_ram[mem->address_bus + 3] << 24) |                         \
                  (_ram[mem->address_bus + 2] << 16) |                         \
                  (_ram[mem->address_bus + 1] << 8) |                          \
                  (_ram[mem->address_bus]);

#define MEM_READ16(_ram)                                                       \
  mem->data_bus = (_ram[mem->address_bus + 1] << 8) | (_ram[mem->address_bus]);

#define MEM_READ8(_ram) mem->data_bus = _ram[mem->address_bus];

void memory_init(Memory *mem) {
  mem->mem_table[0] = mem->wram;
  mem->mem_table[1] = mem->iwram;
  mem->mem_table[2] = mem->io_ram;
  mem->mem_table[3] = mem->palette_ram;
  mem->mem_table[4] = mem->vram;
  mem->mem_table[5] = mem->oam;
  // game pak mem_table and sram table are filled by rom loader
  mem->mem_table[6] = mem->game_pak_rom;
  mem->mem_table[7] = mem->game_pak_rom;
  mem->mem_table[8] = mem->game_pak_rom;
  mem->mem_table[9] = mem->game_pak_rom;
  mem->mem_table[10] = mem->game_pak_rom;
  mem->mem_table[11] = mem->game_pak_rom;
  mem->mem_table[12] = mem->sram;

  mem->mem_mirrors[0] = 0x40000;
  mem->mem_mirrors[1] = 0x8000;
  mem->mem_mirrors[2] = 0x400;
  mem->mem_mirrors[3] = 0x400;
  mem->mem_mirrors[4] = 0x18000;
  mem->mem_mirrors[5] = 0x400;
  // game pak mirrors and sram are filled by rom loader
  mem->mem_mirrors[6] = 0x2000000;
  mem->mem_mirrors[7] = 0x2000000;
  mem->mem_mirrors[8] = 0x2000000;
  mem->mem_mirrors[9] = 0x2000000;
  mem->mem_mirrors[10] = 0x2000000;
  mem->mem_mirrors[11] = 0x2000000;
  mem->mem_mirrors[12] = 0x010000;
}

void mem_write32(Memory *mem) {
  // if (mem->address_bus < 0x4000) {
  //   // bios rom
  // }
  if (mem->address_bus >= 0x2000000 && mem->address_bus <= 0x203FFFF) {
    mem->address_bus = mem->address_bus & 0x3FFFF;

    // WRAM - on-board Work RAM
    MEM_WRITE32(mem->wram);
  } else if (mem->address_bus >= 0x3000000 && mem->address_bus <= 0x3007FFF) {
    // WRAM - On-chip work RAM
    mem->address_bus = mem->address_bus & 0x7FFF;
    MEM_WRITE32(mem->iwram);

  } else if (mem->address_bus >= 0x4000000 && mem->address_bus <= 0x40003FF) {
    // IO Registers
    mem->address_bus = mem->address_bus & 0x3FF;
    MEM_WRITE32(mem->io_ram);

  } else if (mem->address_bus >= 0x5000000 && mem->address_bus <= 0x50003FF) {
    // BG/OBJ Palette RAM
    mem->address_bus = mem->address_bus & 0x3FF;
    MEM_WRITE32(mem->palette_ram);

  } else if (mem->address_bus >= 0x6000000 && mem->address_bus <= 0x6017FFF) {
    // VRAM - Video RAM
    mem->address_bus = mem->address_bus % 0x18000;
    MEM_WRITE32(mem->vram);

  } else if (mem->address_bus >= 0x7000000 && mem->address_bus <= 0x70003FF) {
    // OAM- OBJ Attributes
    mem->address_bus = mem->address_bus & 0x3FF;
    MEM_WRITE32(mem->oam);
  } else if (mem->address_bus >= 0x8000000 && mem->address_bus <= 0x9FFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;
    // 0x1FFFFFF

  } else if (mem->address_bus >= 0xA000000 && mem->address_bus <= 0xBFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;
    // 0x1FFFFFF

  } else if (mem->address_bus >= 0xC000000 && mem->address_bus <= 0xDFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;
    // 0x1FFFFFF

  } else if (mem->address_bus >= 0xE000000 && mem->address_bus <= 0xE00FFFF) {
    mem->address_bus = mem->address_bus & 0xFFFF;
    // 0xFFFF
    // Game Pak SRAM
  }
  // unused
}


void mem_write16(Memory *mem) {

  if (mem->address_bus >= 0x2000000 && mem->address_bus <= 0x203FFFF) {

    mem->address_bus = mem->address_bus & 0x3FFFF;
    // WRAM - on-board Work RAM
    MEM_WRITE16(mem->wram);
  } else if (mem->address_bus >= 0x3000000 && mem->address_bus <= 0x3007FFF) {
    mem->address_bus = mem->address_bus & 0x7FFF;
    // WRAM - On-chip work RAM
    MEM_WRITE16(mem->iwram);

  } else if (mem->address_bus >= 0x4000000 && mem->address_bus <= 0x40003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // IO Registers
    MEM_WRITE16(mem->io_ram);

  } else if (mem->address_bus >= 0x5000000 && mem->address_bus <= 0x50003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // BG/OBJ Palette RAM
    MEM_WRITE16(mem->palette_ram);

  } else if (mem->address_bus >= 0x6000000 && mem->address_bus <= 0x6017FFF) {
    mem->address_bus = mem->address_bus % 0x18000;
    // VRAM - Video RAM
    MEM_WRITE16(mem->vram);

  } else if (mem->address_bus >= 0x7000000 && mem->address_bus <= 0x70003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // OAM- OBJ Attributes
    MEM_WRITE16(mem->oam);
  } else if (mem->address_bus >= 0x8000000 && mem->address_bus <= 0x9FFFFFF) {
    
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xA000000 && mem->address_bus <= 0xBFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xC000000 && mem->address_bus <= 0xDFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xE000000 && mem->address_bus <= 0xE00FFFF) {
    mem->address_bus = mem->address_bus & 0xFFFF;
    // Game Pak SRAM
  }
}

void mem_write8(Memory *mem) {

  if (mem->address_bus >= 0x2000000 && mem->address_bus <= 0x203FFFF) {

    mem->address_bus = mem->address_bus & 0x3FFFF;
    // WRAM - on-board Work RAM
    MEM_WRITE8(mem->wram);
  } else if (mem->address_bus >= 0x3000000 && mem->address_bus <= 0x3007FFF) {
    mem->address_bus = mem->address_bus & 0x7FFF;
    // WRAM - On-chip work RAM
    MEM_WRITE8(mem->iwram);

  } else if (mem->address_bus >= 0x4000000 && mem->address_bus <= 0x40003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // IO Registers
    MEM_WRITE8(mem->io_ram);

  } else if (mem->address_bus >= 0xE000000 && mem->address_bus <= 0xE00FFFF) {
    mem->address_bus = mem->address_bus & 0xFFFF;
    // Game Pak SRAM
  }
}


uint32_t mem_read32(Memory *mem) {

  if (mem->address_bus < 0x4000) {
    // bios rom
    mem->address_bus = mem->address_bus & 0x3FFF;
    MEM_READ32(mem->bios_rom);

  } else if (mem->address_bus >= 0x2000000 && mem->address_bus <= 0x203FFFF) {
    mem->address_bus = mem->address_bus & 0x3FFFF;

    // WRAM - on-board Work RAM
    MEM_READ32(mem->wram);
  } else if (mem->address_bus >= 0x3000000 && mem->address_bus <= 0x3007FFF) {
    mem->address_bus = mem->address_bus & 0x7FFF;
    // WRAM - On-chip work RAM
    MEM_READ32(mem->iwram);

  } else if (mem->address_bus >= 0x4000000 && mem->address_bus <= 0x40003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // IO Registers
    MEM_READ32(mem->io_ram);

  } else if (mem->address_bus >= 0x5000000 && mem->address_bus <= 0x50003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // BG/OBJ Palette RAM
    MEM_READ32(mem->palette_ram);

  } else if (mem->address_bus >= 0x6000000 && mem->address_bus <= 0x6017FFF) {
    mem->address_bus = mem->address_bus % 0x18000;
    // VRAM - Video RAM
    MEM_READ32(mem->vram);

  } else if (mem->address_bus >= 0x7000000 && mem->address_bus <= 0x70003FF) {
    // OAM- OBJ Attributes
    mem->address_bus = mem->address_bus & 0x3FF;
    MEM_READ32(mem->oam);
  } else if (mem->address_bus >= 0x8000000 && mem->address_bus <= 0x9FFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xA000000 && mem->address_bus <= 0xBFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xC000000 && mem->address_bus <= 0xDFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xE000000 && mem->address_bus <= 0xE00FFFF) {
    mem->address_bus = mem->address_bus & 0xFFFF;
    // Game Pak SRAM
  }
  return mem->data_bus;
}

uint32_t mem_read16(Memory *mem) {

  if (mem->address_bus < 0x4000) {
    // bios rom
    mem->address_bus = mem->address_bus & 0x3FFF;
    MEM_READ16(mem->bios_rom);

  } else if (mem->address_bus >= 0x2000000 && mem->address_bus <= 0x203FFFF) {

    mem->address_bus = mem->address_bus & 0x3FFFF;
    // WRAM - on-board Work RAM
    MEM_READ16(mem->wram);
  } else if (mem->address_bus >= 0x3000000 && mem->address_bus <= 0x3007FFF) {
    // WRAM - On-chip work RAM
    mem->address_bus = mem->address_bus & 0x7FFF;
    MEM_READ16(mem->iwram);

  } else if (mem->address_bus >= 0x4000000 && mem->address_bus <= 0x40003FF) {
    // IO Registers
    mem->address_bus = mem->address_bus & 0x3FF;
    MEM_READ16(mem->io_ram);

  } else if (mem->address_bus >= 0x5000000 && mem->address_bus <= 0x50003FF) {
    // BG/OBJ Palette RAM
    mem->address_bus = mem->address_bus & 0x3FF;
    MEM_READ16(mem->palette_ram);

  } else if (mem->address_bus >= 0x6000000 && mem->address_bus <= 0x6017FFF) {
    mem->address_bus = mem->address_bus % 0x18000;
    // VRAM - Video RAM
    MEM_READ16(mem->vram);

  } else if (mem->address_bus >= 0x7000000 && mem->address_bus <= 0x70003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // OAM- OBJ Attributes
    MEM_READ16(mem->oam);
  } else if (mem->address_bus >= 0x8000000 && mem->address_bus <= 0x9FFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xA000000 && mem->address_bus <= 0xBFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xC000000 && mem->address_bus <= 0xDFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xE000000 && mem->address_bus <= 0xE00FFFF) {
    mem->address_bus = mem->address_bus & 0xFFFF;
    // Game Pak SRAM
  }
  return mem->data_bus;
}


uint32_t mem_read8(Memory *mem) {

  if (mem->address_bus < 0x4000) {
    // bios rom
    mem->address_bus = mem->address_bus & 0x3FFF;
    MEM_READ8(mem->bios_rom);

  } else if (mem->address_bus >= 0x2000000 && mem->address_bus <= 0x203FFFF) {
    mem->address_bus = mem->address_bus & 0x3FFFF;

    // WRAM - on-board Work RAM
    MEM_READ8(mem->wram);
  } else if (mem->address_bus >= 0x3000000 && mem->address_bus <= 0x3007FFF) {
    mem->address_bus = mem->address_bus & 0x7FFF;
    // WRAM - On-chip work RAM
    MEM_READ8(mem->iwram);

  } else if (mem->address_bus >= 0x4000000 && mem->address_bus <= 0x40003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // IO Registers
    MEM_READ8(mem->io_ram);

  } else if (mem->address_bus >= 0x5000000 && mem->address_bus <= 0x50003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // BG/OBJ Palette RAM
    MEM_READ8(mem->palette_ram);

  } else if (mem->address_bus >= 0x6000000 && mem->address_bus <= 0x6017FFF) {
    mem->address_bus = mem->address_bus % 0x18000;
    // VRAM - Video RAM
    MEM_READ8(mem->vram);

  } else if (mem->address_bus >= 0x7000000 && mem->address_bus <= 0x70003FF) {
    mem->address_bus = mem->address_bus & 0x3FF;
    // OAM- OBJ Attributes
    MEM_READ8(mem->oam);
  } else if (mem->address_bus >= 0x8000000 && mem->address_bus <= 0x9FFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xA000000 && mem->address_bus <= 0xBFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xC000000 && mem->address_bus <= 0xDFFFFFF) {
    mem->address_bus = mem->address_bus & 0x1FFFFFF;

  } else if (mem->address_bus >= 0xE000000 && mem->address_bus <= 0xE00FFFF) {
    mem->address_bus = mem->address_bus & 0xFFFF;
    // Game Pak SRAM
  }
  return mem->data_bus;
}

#undef MEM_WRITE32
#undef MEM_WRITE16
#undef MEM_WRITE8
#undef MEM_READ32
#undef MEM_READ16
#undef MEM_READ8
