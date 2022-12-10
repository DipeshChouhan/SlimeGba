#include "gba.h"
#include "../arm/arm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int processor_modes[16] = {USR, FIQ, IRQ, SVC, 7, 7, 7, ABT,
                           7,   7,   7,   UND, 7, 7, 7, SYS};

void power_on_gba(uint8_t *rom, unsigned int rom_size) {
  // printf("file open of size - %d\n", rom_size);
  Gba gba;
  gba.arm.curr_instruction = 0x03000000;
  printf("%d\n", gba.arm.curr_instruction);
  init_arm(&gba.arm);

  memcpy(&gba.memory.iwram, rom, rom_size);
  free(rom);
  int index = 0;
  int total = 0;
  gba.arm.mode = SVC;
  gba.arm.cpsr = 51;
  gba.arm.state = THUMB_STATE;

  while (index < rom_size) {
    if (gba.arm.state == THUMB_STATE) {
      thumb_exec(&gba.arm);
      index += 2;
    } else {
      arm_exec(&gba.arm);
      printf("called\n");
      index += 4;
    }
    ++total;
  }
  printf("called %d\n", total);
  if (gba.arm.mode == UND) {
    printf("exit-code: %d\n", gba.arm.general_regs[1]);
    exit(0);
  }
}
