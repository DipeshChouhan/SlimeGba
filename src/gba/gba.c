#include "gba.h"
#include "../arm/arm.h"
#include <stdio.h>


void power_on_gba(uint8_t *rom, unsigned int rom_size) {
  // printf("file open of size - %d\n", rom_size);
  Arm arm;


  
  printf("rom size %d\n", rom_size);
  for (int i = 0; i < rom_size; i+=4) {
    arm.data_bus = ((rom[i + 3] << 24) | (rom[i + 2] << 16) | (rom[i + 1] << 8) | 
        (rom[i]));
    printf("%0X\n", arm.data_bus);
    arm_exec(&arm);
  }
}
