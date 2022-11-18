
#include "memory.h"

/*General Internal Memory
  00000000-00003FFF   BIOS - System ROM         (16 KBytes)
  00004000-01FFFFFF   Not used
  02000000-0203FFFF   WRAM - On-board Work RAM  (256 KBytes) 2 Wait
  02040000-02FFFFFF   Not used
  03000000-03007FFF   WRAM - On-chip Work RAM   (32 KBytes)
  03008000-03FFFFFF   Not used
  04000000-040003FE   I/O Registers
  04000400-04FFFFFF   Not used
Internal Display Memory
  05000000-050003FF   BG/OBJ Palette RAM        (1 Kbyte)
  05000400-05FFFFFF   Not used
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
  10000000-FFFFFFFF   Not used (upper 4bits of address bus unused)
  */
void mem_write32(uint32_t address, uint32_t value) {
  if (address < 0x4000) {
    // bios rom
  } else if (address >= 0x2000000 && address <= 0x203FFFF) {
    

  } else if (address >= 0x3000000 && address <= 0x3007FFF) {

  } else if (address >= 0x4000000 && address <= 0x40003FE) {

  } else if (address >= 0x5000000 && address <= 0x50003FF) {

  } else if (address >= 0x6000000 && address <= 0x6017FFF) {

  } else if (address >= 0x7000000 && address <= 0x70003FF) {

  } else if (address >= 0x8000000 && address <= 0x9FFFFFF) {

  } else if (address >= 0xA000000 && address <= 0xBFFFFFF) {

  } else if (address >= 0xC000000 && address <= 0xDFFFFFF) {

  } else if (address >= 0xE000000 && address <= 0xE00FFFF) {

  }

  // unused
}

uint32_t mem_read32(uint32_t address) { return 0; }
