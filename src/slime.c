#include <stdio.h>

#include <stdlib.h>
#include "utils/file_open.h"
#include "arm/disassembler.h"
#include "gba/gba.h"
#include "../libs/cJSON/cJSON.h"

Disassembler disassembler;
int main(int argc, char **argv) {
  disassembler.log_dir_name = "../logs/"; // path from executable
  disassembler.log_dir_size = 8;

  disassembler.json_obj = cJSON_CreateObject();
  if (disassembler.json_obj == NULL) {
    printf("error creating json_obj\n");
    return 0;
  }

  disassembler.json_arr = cJSON_CreateArray();
  cJSON_AddItemToObject(disassembler.json_obj, "instructions",
                        disassembler.json_arr);

  create_json_log_file("basic.asm");
  if (argc < 2) {
    printf("please provide gba rom\n");
    return 0;
  }
  printf("File: %s\n", argv[1]);
  unsigned int rom_size = 0;
  uint8_t *rom = load_binary_file(argv[1], &rom_size);
  if (rom == NULL) {
    return 0;
  }

  power_on_gba(rom, rom_size);
  printf("%s\n", cJSON_Print(disassembler.json_obj));
  char *json_data = cJSON_Print(disassembler.json_obj);
  fputs(json_data, disassembler.json_log_file);
  cJSON_Delete(disassembler.json_obj);
  fclose(disassembler.json_log_file);
  free(json_data);
  return 0;
}
