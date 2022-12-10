#include "disassembler.h"
#include "arm.h"
#include "string.h"
#include <stdio.h>

void create_json_log_file(char *bin_file_name) {
  int file_size = 0;
  int index = 0;
  while (bin_file_name[index] != '.') {
    ++file_size;
    ++index;
  }

  char file_name[disassembler.log_dir_size + file_size + 6];
  const char *temp = ".json";
  memcpy(file_name, disassembler.log_dir_name, disassembler.log_dir_size);
  memcpy(&file_name[disassembler.log_dir_size], bin_file_name, file_size);
  file_size += disassembler.log_dir_size;
  memcpy(&file_name[file_size], temp, 6);

  FILE* file = fopen(file_name, "w");
  if (file == NULL) {
    printf("failed to open %s file\n", file_name);
  }

  disassembler.json_log_file = file;
}

void write_decoder_log(Arm *arm, char *name) {
  cJSON *decode_str = cJSON_CreateString(name);
  cJSON_AddItemToArray(disassembler.json_arr, decode_str);
}

void write_instruction_log(Arm *arm, char *name) {

  cJSON *inst = cJSON_CreateObject();
  cJSON *inst_name = cJSON_CreateString(name);
  cJSON *r0 = cJSON_CreateNumber(arm->general_regs[0]);
  cJSON *r1 = cJSON_CreateNumber(arm->general_regs[1]);
  cJSON *r2 = cJSON_CreateNumber(arm->general_regs[2]);
  cJSON *r3 = cJSON_CreateNumber(arm->general_regs[3]);
  cJSON *r4 = cJSON_CreateNumber(arm->general_regs[4]);
  cJSON *r5 = cJSON_CreateNumber(arm->general_regs[5]);
  cJSON *r6 = cJSON_CreateNumber(arm->general_regs[6]);
  cJSON *r7 = cJSON_CreateNumber(arm->general_regs[7]);
  cJSON *r8 = cJSON_CreateNumber(arm->general_regs[8]);
  cJSON *r9 = cJSON_CreateNumber(arm->general_regs[9]);
  cJSON *r10 = cJSON_CreateNumber(arm->general_regs[10]);
  cJSON *r11 = cJSON_CreateNumber(arm->general_regs[11]);
  cJSON *r12 = cJSON_CreateNumber(arm->general_regs[12]);
  cJSON *r13 = cJSON_CreateNumber(arm->general_regs[13]);
  cJSON *r14 = cJSON_CreateNumber(arm->svc_regs[1]);
  cJSON *r15 = cJSON_CreateNumber(arm->general_regs[15]);
  cJSON *cpsr = cJSON_CreateNumber(arm->cpsr);
  cJSON *curr_instruction = cJSON_CreateNumber(arm->curr_instruction - ((arm->state == THUMB_STATE) ? 2 : 4));

  cJSON_AddItemToObject(inst, "instruction", inst_name);
  cJSON_AddItemToObject(inst, "r0", r0);
  cJSON_AddItemToObject(inst, "r1", r1);
  cJSON_AddItemToObject(inst, "r2", r2);
  cJSON_AddItemToObject(inst, "r3", r3);
  cJSON_AddItemToObject(inst, "r4", r4);
  cJSON_AddItemToObject(inst, "r5", r5);
  cJSON_AddItemToObject(inst, "r6", r6);
  cJSON_AddItemToObject(inst, "r7", r7);
  cJSON_AddItemToObject(inst, "r8", r8);
  cJSON_AddItemToObject(inst, "r9", r9);
  cJSON_AddItemToObject(inst, "r10", r10);
  cJSON_AddItemToObject(inst, "r11", r11);
  cJSON_AddItemToObject(inst, "r12", r12);
  cJSON_AddItemToObject(inst, "r13", r13);
  cJSON_AddItemToObject(inst, "r14", r14);
  cJSON_AddItemToObject(inst, "r15", r15);
  cJSON_AddItemToObject(inst, "cpsr", cpsr);
  cJSON_AddItemToObject(inst, "curr_inst", curr_instruction);

  cJSON_AddItemToObject(disassembler.json_arr, "instruction", inst);
  
}
