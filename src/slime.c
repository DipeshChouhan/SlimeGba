#include <stdio.h>
#include "gba/gba.h"
#include "utils/file_open.h"
int main(int argc, char **argv) {
    if (argc < 2) {
      printf("Please provide GBA rom\n");
      return 0;
    }
      unsigned int rom_size;
      uint8_t *rom = load_binary_file(argv[1], &rom_size);
      if (rom == NULL) {
        printf("Make sure rom is correct\n");
        return 0;
      }
      power_on_gba(rom, rom_size);
    return 0;
}
