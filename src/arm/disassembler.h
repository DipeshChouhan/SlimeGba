#ifndef _DISASSEMBLER_H
#define _DISASSEMBLER_H
#include <stdio.h>
#include "../../libs/cJSON/cJSON.h"
#include "arm.h"

typedef struct Disassembler{

  FILE *json_log_file;
  char *log_dir_name;
  int log_dir_size;
  cJSON *json_obj;
  cJSON *json_arr;
  
} Disassembler;

extern Disassembler disassembler;

void create_json_log_file(char *bin_file_name);

void write_instruction_log(Arm *arm, char *name);
void write_decoder_log(Arm *arm, char *name);

#endif
