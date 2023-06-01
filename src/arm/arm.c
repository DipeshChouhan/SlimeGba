/* TODO - Check decoder code */
// TODO - test rotate right and shifter operand value   {DONE}
// TODO seprate flag setting and cpsr = spsr in data processing instructions
// {DONE}
// TODO CPSR = SPSR code is incorrect Check again {DONE}
// TODO optimize overflow flag setting  {DONE}
// TODO choose registers based on mode in instructions  !{DONE - NOT TESTED}
// TODO remove checking condition passed !{DONE}
//
// TODO implement memory read and write change in all load and store
// !{DONE Not Tested}
// TODO flag setting for multiply instructions {DONE Not Tested}
// TODO check flag setting for data processing instructions !{DONE}
// TODO check overflow flag setting in subtraction !{DONE}
// TODO Check singned multiply !{DONE}
// TODO check msr instruction implementation !{Done}
// TODO set processor mode in msr instruction !{Done}
// TODO check when to switch between arm and thumb state in instructions
// TODO implement B, BL correctly {DONE}
// TODO check fetch implementation ![DONE Not Tested]
// TODO check condition field implementation !{DONE Not Tested}
// TODO implement miscellaneous loads and store instructions  !{DONE - NOT
// TESTED}
// TODO check sign extending in miscellaneous loads and store instruction
// !{DONE}
// TODO correct the goto LOAD_STORE_W_U_INSTS implementation !{DONE Not Tested}
//
// TODO check STRBT like instruction which access as if in user mode
// !{IMPORTANT - Research Required}

// TODO check sign extending in all instructions !{DONE}
// TODO Fix error in all Multiply instruction !{DONE check one more time}

// TODO check operand calculation in decode code for all instruction formats
// !{DONE}

// TODO check overflow flag setting !{DONE}

#include "arm.h"
#include "../gba/gba.h"
#include "../memory/memory.h"
#include "arm_inst_decode.h"
#include "disassembler.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define DEBUG_ON

#define R_15 15
#define U_BIT (OP_CODE & 0x800000)
#define S_BIT (OP_CODE & 0x100000)

#define RN_C ((OP_CODE >> 16) & 0xF)
#define RD_C ((OP_CODE >> 12) & 0xF)
#define RM_C (OP_CODE & 0xF)
#define RS_C ((OP_CODE >> 8) & 0xF)
#define SHIFT_IMM ((OP_CODE >> 7) & 0x1F)
#define ROTATE_IMM ((OP_CODE >> 8) & 0xF)
#define IMM_8 (OP_CODE & 0xFF)

int processor_modes[16] = {USR, FIQ, IRQ, SVC, 7, 7, 7, ABT,
                           7,   7,   7,   UND, 7, 7, 7, SYS};

// #define DATA_PROCESS_NZCV(_arm)                                                \
//   temp = _arm->cpsr & 0xFFFFFFF;                                               \
//   temp |= (result & 0x80000000);                                               \
//   temp |= ((*reg_p == 0) << 30);                                               \
//   temp |= ((result & 0x100000000) >> 3);                                       \
//   temp |= ((((rn & 0x80000000) == (shifter_operand & 0x80000000)) &&           \
//             ((rn & 0x80000000) != (result & 0x80000000)))                      \
//            << 28);

// TODO:
#define FLAG_SETTING_NZCV(_nf, _zf, _cf)                                       \
  temp = arm->cpsr & 0xFFFFFFF;                                                \
  temp |= (_nf & 0x80000000);                                                  \
  temp |= ((_zf == 0) << 30);                                                  \
  temp |= (_cf);                                                               \
  temp |= ((((rn & 0x80000000) == (shifter_operand & 0x80000000)) &&           \
            ((rn & 0x80000000) != (result & 0x80000000)))                      \
           << 28);                                                             \
  arm->cpsr = temp;

#define DATA_PROCESS_NZCV()                                                    \
  FLAG_SETTING_NZCV(result, *reg_p, (result & 0x100000000) >> 3)

#define FLAG_SETTING_NZC(_nf, _zf, _cf)                                        \
  temp = arm->cpsr & 0x1FFFFFFF;                                               \
  temp |= (_nf & 0x80000000);                                                  \
  temp |= ((_zf == 0) << 30);                                                  \
  temp |= (_cf << 29);                                                         \
  arm->cpsr = temp;

#define DATA_PROCESS_NZC() FLAG_SETTING_NZC(*reg_p, *reg_p, shifter_carry_out);

#define FLAG_SETTING_NZ(_nf, _zf)                                              \
  temp = arm->cpsr & 0x3FFFFFFF;                                               \
  temp |= (_nf & 0x80000000);                                                  \
  temp |= ((_zf == 0) << 30);                                                  \
  arm->cpsr = temp;

#define COMPARE_INSTS_NZC()                                                    \
  FLAG_SETTING_NZC(result, (result & 0xFFFFFFFF), shifter_carry_out);

#define COMPARE_INSTS_NZCV()                                                   \
  FLAG_SETTING_NZCV(result, (result & 0xFFFFFFFF), (result & 0x100000000) >> 3)

#define MUL_NZ(_arm)                                                           \
  temp = _arm->cpsr & 0x3FFFFFFF;                                              \
  temp |= (*reg_p & 0x80000000);                                               \
  temp |= ((*reg_p == 0) << 30);                                               \
  _arm->cpsr = temp;

#define USMULL_NZ(_arm)                                                        \
  temp = _arm->cpsr & 0x3FFFFFFF;                                              \
  temp |= (*reg_p & 0x80000000);                                               \
  temp |= (((*reg_p == 0) && (*_arm->reg_table[rn] == 0)) << 30);              \
  _arm->cpsr = temp;

#define DATA_PROCESS_RD_EQ_R15(_arm)                                           \
  if (rd == R_15) {                                                            \
    _arm->curr_instruction = *reg_p;                                           \
    if (s_bit) {                                                               \
      if (arm->mode > 1) {                                                     \
        _arm->cpsr = _arm->spsr[_arm->mode - 2];                               \
        _arm->mode = processor_modes[_arm->cpsr & 0xF];                        \
      }                                                                        \
      write_instruction_log(_arm, "SUB");                                      \
      goto END;                                                                \
    }                                                                          \
  }

#define OP_CODE arm->data_bus

// not considering _ror is zero

// TODO Implement this function
// uint32_t arm_read(uint32_t addr) { return 0; }
//
// void arm_write(uint32_t addr, uint32_t value);

#define ARM_WRITE(_address, _data, _type)                                      \
  ((Gba *)arm)->memory.address_bus = _address;                                 \
  ((Gba *)arm)->memory.data_bus = _data;                                       \
  _type(&((Gba *)arm)->memory);

#define ARM_READ(_address, _dest, _type)                                       \
  ((Gba *)arm)->memory.address_bus = _address;                                 \
  _dest = _type(&((Gba *)arm)->memory);

#define ARM_FETCH(_address, _dest)                                             \
  ARM_READ(_address, _dest, mem_read32);                                       \
  arm->general_regs[R_15] = _address + 8;                                      \
  _address += 4;

// algorithm to generate table for all registers
void gen_reg_table(Arm *arm) {

  for (int mode = 0; mode < 7; mode++) {
    for (int reg = 0; reg < 16; reg++) {
      if (mode < 2) {
        // user mode and system mode
        arm->reg_table[(mode * 16) + reg] = &arm->general_regs[reg];
        continue;
      }
      if (mode == FIQ) {
        // fiq mode
        if (reg >= 8 && reg <= 14) {
          arm->reg_table[(mode * 16) + reg] = &arm->fiq_regs[reg - 8];
          continue;
        }
        arm->reg_table[(mode * 16) + reg] = &arm->general_regs[reg];
        continue;
      }

      if (reg == 13 || reg == 14) {
        if (mode == IRQ) {
          arm->reg_table[(mode * 16) + reg] = &arm->irq_regs[reg - 13];

        } else if (mode == SVC) {
          arm->reg_table[(mode * 16) + reg] = &arm->svc_regs[reg - 13];

        } else if (mode == ABT) {
          arm->reg_table[(mode * 16) + reg] = &arm->abt_regs[reg - 13];

        } else {
          // und
          arm->reg_table[(mode * 16) + reg] = &arm->und_regs[reg - 13];
        }
        continue;
      }

      arm->reg_table[(mode * 16) + reg] = &arm->general_regs[reg];
    }
  }
}

void init_arm(Arm *arm) {

  int index = 0;

  // initializing registers
  while (index < 16) {
    arm->general_regs[index] = 0;
    if (index < 7) {
      arm->fiq_regs[index] = 0;
      if (index < 2) {
        arm->svc_regs[index] = 0;
        arm->abt_regs[index] = 0;
        arm->irq_regs[index] = 0;
        arm->und_regs[index] = 0;
      }
    }
    ++index;
  }

  arm->cpsr = 0;
  arm->exception_gen = 1;
  arm->reset_pin = 1;
  arm->irq_pin = 0;
  arm->fiq_pin = 0;

  index = 0;
  while (index < 5) {
    arm->spsr[index++] = 16;
  }
  gen_reg_table(arm);

  // all global test goes here
#ifdef DEBUG_ON
  uint32_t test_val = ROTATE_RIGHT32(0x3F, 0xE * 2);
  uint64_t test_val64 = 0;
  assert(ROTATE_RIGHT32(0x3F, 0xE * 2) == 0x3F0);
  assert(ROTATE_RIGHT32(0xFC, 0xF * 2) == 0x3F0);

  assert(IS_BIT_SET(0xFF30, 10) == 1);
  assert(IS_BIT_SET(0x8800FF30, 31) == 1);
  assert(IS_BIT_SET(0x0, 31) == 0);

  assert(IS_BIT_NOT_SET(0xFF30, 10) == 0);
  assert(IS_BIT_NOT_SET(0x8800FF30, 31) == 0);
  assert(IS_BIT_NOT_SET(0x0, 31) == 1);

  test_val = 0x8800FF30;
  assert(SET_BIT(test_val, 31, 0) == 0x800FF30);

  assert(SET_BIT(test_val, 28, 1) == 0x9800FF30);

  test_val = 0x0;
  assert(SET_BIT(test_val, 31, 1) == 0x80000000);

  test_val64 = 0x259840FF30;
  assert(GET_BIT(test_val64, 37) == 1);
  assert(GET_BIT(test_val64, 0) == 0);
  assert(GET_BIT(test_val64, 32) == 1);
  test_val = arm->cpsr;
  test_val = SET_BIT(test_val, 7, 1);
  assert(IS_BIT_SET(test_val, 7) == 1);
  test_val = SET_BIT(test_val, 6, 1);
  assert(IS_BIT_SET(test_val, 6));
  test_val = SET_BIT(test_val, 7, 0);
  assert(IS_BIT_NOT_SET(test_val, 7) == 1);

  test_val = arm->cpsr;
  DISABLE_FIQ(test_val);
  assert(IS_BIT_SET(test_val, 6) == 1);
  DISABLE_IRQ(test_val);
  assert(IS_BIT_NOT_SET(test_val, 7) == 0);

  assert(ASR_32(0x80000000, 10) == 0xFFE00000);
  assert(ASR_32(0xFFFFFFFF, 5) == 0xFFFFFFFF);
  assert(ASR_32(0xF0, 4) == 0xF);
  assert(ASR_32(0xF0, 8) == 0x0);
  assert(ASR_32(0x80000000, 31) == 0xFFFFFFFF);
  assert(ASR_SIGN_32(0x80000000, 31, 1) == 0xFFFFFFFF);
  assert(ROTATE_RIGHT32(0x3, 8) == 0x3000000);

#endif
}

int arm_exec(Arm *arm) {
#ifdef DEBUG_ON
  if (arm->mode == UND) {

    printf("%s\n", cJSON_Print(disassembler.json_obj));
    char *json_data = cJSON_Print(disassembler.json_obj);
    fputs(json_data, disassembler.json_log_file);
    cJSON_Delete(disassembler.json_obj);
    fclose(disassembler.json_log_file);
    free(json_data);
    printf("exit code - %d\n", arm->general_regs[0]);
    exit(0);
  }
#endif

  uint32_t temp = 0;
  uint32_t shifter_operand = 0;
  int shifter_carry_out = 0;
  uint32_t ls_address = 0; // load store address
  uint32_t rm = 0;
  uint32_t rs = 0;
  uint32_t rn = 0;
  uint32_t *reg_p = NULL;
  uint32_t rd = 0;
  int reg_count = 0;
  int s_bit = 0;
  uint32_t shift = 0;
  uint32_t cpu_cycle = 0;

  uint32_t shift_imm = 0;
  uint32_t rotate_imm = 0;
  uint64_t result = 0;

  static void *dp_inst_table[] = {
      &&AND_INST, &&EOR_INST, &&SUB_INST, &&RSB_INST, &&ADD_INST, &&ADC_INST,
      &&SBC_INST, &&RSC_INST, &&TST_INST, &&TEQ_INST, &&CMP_INST, &&CMN_INST,
      &&ORR_INST, &&MOV_INST, &&BIC_INST, &&MVN_INST};

  static void *mul_inst_table[] = {&&MUL_INST,   &&MLA_INST,   &&UNDEFINED,
                                   &&UNDEFINED,  &&UMULL_INST, &&UMLAL_INST,
                                   &&SMULL_INST, &&SMLAL_INST};

  static void *cond_field_table[] = {
      &&CHECK_EQ, &&CHECK_NE, &&CHECK_CS_HS, &&CHECK_CC_LO,
      &&CHECK_MI, &&CHECK_PL, &&CHECK_VS,    &&CHECK_VC,
      &&CHECK_HI, &&CHECK_LS, &&CHECK_GE,    &&CHECK_LT,
      &&CHECK_GT, &&CHECK_LE, &&CHECK_AL,    &&UNCONDITIONAL};

  INTERRUPT_REQUEST();

FETCH:
  // fetch
  ARM_FETCH(arm->curr_instruction, arm->data_bus);
  // ---

  // goto DECODE;

  goto *cond_field_table[OP_CODE >> 28];

// check for conditions and set execute_instruction to 1
CHECK_EQ:
  if (IS_BIT_SET(arm->cpsr, ZF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_NE:
  if (IS_BIT_SET(arm->cpsr, ZF_BIT)) {
    goto END;
  }
  goto DECODE;
CHECK_CS_HS:
  if (IS_BIT_SET(arm->cpsr, CF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_CC_LO:
  if (IS_BIT_SET(arm->cpsr, CF_BIT)) {
    goto END;
  }
  goto DECODE;
CHECK_MI:
  if (IS_BIT_SET(arm->cpsr, NF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_PL:
  if (IS_BIT_SET(arm->cpsr, NF_BIT)) {
    goto END;
  }
  goto DECODE;
CHECK_VS:
  if (IS_BIT_SET(arm->cpsr, VF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_VC:
  if (IS_BIT_SET(arm->cpsr, VF_BIT)) {
    goto END;
  }
  goto DECODE;
CHECK_HI:
  // c set and z clear
  if ((arm->cpsr & 0x60000000) == 0x20000000) {
    goto DECODE;
  }
  goto END;
CHECK_LS:
  if ((IS_BIT_SET(arm->cpsr, CF_BIT) == 0) || IS_BIT_SET(arm->cpsr, ZF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_GE:
  if (GET_BIT(arm->cpsr, NF_BIT) == GET_BIT(arm->cpsr, VF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_LT:
  if (GET_BIT(arm->cpsr, NF_BIT) != GET_BIT(arm->cpsr, VF_BIT)) {
    goto DECODE;
  }
  goto END;
CHECK_GT:
  if ((IS_BIT_SET(arm->cpsr, ZF_BIT) == 0) &&
      (GET_BIT(arm->cpsr, NF_BIT) == GET_BIT(arm->cpsr, VF_BIT))) {
    goto DECODE;
  }
  goto END;
CHECK_LE:
  if (IS_BIT_SET(arm->cpsr, ZF_BIT) ||
      (GET_BIT(arm->cpsr, NF_BIT) != GET_BIT(arm->cpsr, VF_BIT))) {
    goto DECODE;
  }
  goto END;
CHECK_AL:
  // alway conditional

// Instruction type decoding
DECODE:

  if ((OP_CODE & MULTIPLY_MASK) == MULTIPLY_DECODE) {
    // multiply instructions

    temp = (OP_CODE >> 21) & 0x7;
    reg_count = arm->mode * 16;
    reg_p = arm->reg_table[reg_count + RN_C];

    rs = *arm->reg_table[reg_count + RS_C];
    rm = *arm->reg_table[reg_count + RM_C];
    rn = reg_count + RD_C;
    s_bit = S_BIT;

    // couting m for cycles
    // TODO - check for correctness

    shifter_operand = rs >> 8;
    shift = rm >> 8;

    if ((shifter_operand == 0x0 || shifter_operand == 0xFFFFFF) &&
        shifter_operand == shift) {
      //[32:8]
      cpu_cycle = 1 + 2;
      goto *mul_inst_table[temp];
    }
    shifter_operand = rs >> 16;
    shift = rm >> 16;
    if ((shifter_operand == 0x0 || shifter_operand == 0xFFFF) &&
        shifter_operand == shift) {
      //[32:16]
      cpu_cycle = 2 + 2;
      goto *mul_inst_table[temp];
    }
    shifter_operand = rs >> 24;
    shift = rm >> 24;
    if ((shifter_operand == 0x0 || shifter_operand == 0xFF) &&
        shifter_operand == shift) {
      // [32:24]
      cpu_cycle = 3 + 2;
      goto *mul_inst_table[temp];
    }

    cpu_cycle = 4 + 2;

    goto *mul_inst_table[temp];
  } else if ((OP_CODE & LOAD_STORE_H_D_S_MASK) == LOAD_STORE_H_D_S_DECODE) {
    // Load and store halfword or doubleword, and load signed byte instructions,
    // and swp and swpb

    temp = OP_CODE & SWP_MASK;
    if (temp == SWP_DECODE) {
      goto SWP_INST;
    } else if (temp == SWPB_DECODE) {
      goto SWPB_INST;
    }
    goto LOAD_STORE_H_D_S;
  } else if ((OP_CODE & DATA_PROCESS_MASK) == DATA_PROCESS_DECODE) {
    // instructions can data processing, control or undefined
    if ((OP_CODE & CONTROL_MASK) != CONTROL_DECODE) {
      // instructions are data processing
      goto DATA_PROCESS;
    }

    goto CONTROL;

    // control instruction

  } else if ((OP_CODE & LOAD_STORE_W_U_MASK) == LOAD_STORE_W_U_DECODE) {
    // Load and store word or unsigned byte instructions and media instructions
    // and architecturally undefined instructions
    goto LOAD_STORE_W_U;
  } else if ((OP_CODE & LOAD_STORE_M_MASK) == LOAD_STORE_M_DECODE) {
    // Load and Store Multiple instructions
    goto LOAD_STORE_M;
  } else if ((OP_CODE & BRANCH_LINK_MASK) == BRANCH_LINK_DECODE) {
    // Branch and branch link instructions
    reg_count = (arm->mode * 16) + 14; // R14 is link register
    goto BRANCH_LINK;
  } else if ((OP_CODE & SWI_MASK) == SWI_DECODE) {
    // swi instruction
    goto SWI;
  } else if ((OP_CODE & COPROCESSOR_MASK) == COPROCESSOR_DECODE) {
    // coprocessor instructions
    goto COPROCESSOR;
  } else {
    // undefined, unimplemented
    goto UNDEFINED;
  }

LOAD_STORE_H_D_S:
#define immedH rotate_imm
#define immedL shift_imm
#define offset_8 shifter_operand
  immedL = RM_C;
  immedH = (OP_CODE >> 8) & 0xF;
  reg_count = arm->mode * 16;
  reg_p = arm->reg_table[reg_count + RN_C];
  temp = OP_CODE & LS_H_D_S_MASK;
  if (temp == LS_H_D_S_REG_DECODE) {

    rm = *arm->reg_table[reg_count + RM_C];
    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }

  } else if (temp == LS_H_D_S_IMM_DECODE) {
    offset_8 = (immedH << 4) | immedL;
    if (U_BIT) {
      ls_address = *reg_p + offset_8;
    } else {
      ls_address = *reg_p - offset_8;
    }

  } else if (temp == LS_H_D_S_IMM_PR_DECODE) {
    offset_8 = (immedH << 4) | immedL;
    if (U_BIT) {
      ls_address = *reg_p + offset_8;
    } else {
      ls_address = *reg_p - offset_8;
    }
    *reg_p = ls_address;

  } else if (temp == LS_H_D_S_REG_PR_DECODE) {
    rm = *arm->reg_table[reg_count + RM_C];
    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }
    *reg_p = ls_address;

  } else if (temp == LS_H_D_S_REG_PO_DECODE) {
    rm = *arm->reg_table[reg_count + RM_C];
    ls_address = *reg_p;
    if (U_BIT) {
      *reg_p = *reg_p + rm;
    } else {
      *reg_p = *reg_p - rm;
    }
  } else if (temp == LS_H_D_S_IMM_PO_DECODE) {
    offset_8 = (immedH << 4) | immedL;
    ls_address = *reg_p;
    if (U_BIT) {
      *reg_p = *reg_p + offset_8;
    } else {
      *reg_p = *reg_p - offset_8;
    }
  }

  goto LOAD_STORE_H_D_S_INSTS;
LOAD_STORE_W_U:
#define OFFSET_12 (OP_CODE & 0xFFF)

  rn = RN_C;
  reg_count = arm->mode * 16;
  reg_p = arm->reg_table[reg_count + rn];
  shifter_operand = OFFSET_12;
  temp = OP_CODE & LS_W_U_IMM_MASK;
  if (temp == LS_W_U_IMM_DECODE) {

    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }
    goto LOAD_STORE_W_U_INSTS;

  } else if (temp == LS_W_U_IMM_PR_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }

    *reg_p = ls_address;
    goto LOAD_STORE_W_U_INSTS;
  }
  temp = OP_CODE & LS_W_U_IMM_PO_MASK;
  if (temp == LS_W_U_IMM_PO_DECODE) {
    ls_address = *reg_p;
    if (U_BIT) {
      *reg_p = *reg_p + shifter_operand;
    } else {
      *reg_p = *reg_p - shifter_operand;
    }

    if (IS_BIT_SET(OP_CODE, 21)) {
      goto LOAD_STORE_W_U_T_INSTS;
    }
    goto LOAD_STORE_W_U_INSTS;
  }

  rm = *arm->reg_table[reg_count + RM_C];
  temp = OP_CODE & LS_W_U_REG_MASK;
  if (temp == LS_W_U_REG_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }

    goto LOAD_STORE_W_U_INSTS;

  } else if (temp == LS_W_U_REG_PR_DECODE) {

    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }

    *reg_p = ls_address;

    goto LOAD_STORE_W_U_INSTS;
  }
  temp = OP_CODE & LS_W_U_REG_PO_MASK;
  if (temp == LS_W_U_REG_PO_DECODE) {

    ls_address = *reg_p;
    if (U_BIT) {
      *reg_p = *reg_p + rm;
    } else {
      *reg_p = *reg_p - rm;
    }

    if (IS_BIT_SET(OP_CODE, 21)) {
      goto LOAD_STORE_W_U_T_INSTS;
    }

    goto LOAD_STORE_W_U_INSTS;
  }
  // TODO check below implementation

  shift_imm = SHIFT_IMM;
  shift = OP_CODE & 0x60;
  temp = OP_CODE & LS_W_U_SCALED_MASK;

  switch (shift) {
  case 0x00: // LSL
    shifter_operand = rm << shift_imm;
    break;
  case 0x20: // LSR
    if (shift_imm == 0) {
      shifter_operand = 0;
    } else {
      shifter_operand = rm >> shift_imm;
    }
    break;

  case 0x40: // ASR
    s_bit = GET_BIT(rm, 31);
    if (shift_imm == 0) {
      shifter_operand = 0xFFFFFFFF * s_bit;
    } else {

      shifter_operand = ASR_SIGN_32(rm, shift_imm, s_bit);
      // shifter_operand =
      //     (rm >> shift_imm) | (s_bit * (0xFFFFFFFF << (32 - shift_imm)));
    }
    break;

  default:
    if (shift_imm == 0) {
      shifter_operand = ((arm->cpsr << 2) & 0x80000000) | (rm >> 1);
    } else {
      shifter_operand = ROTATE_RIGHT32(rm, shift_imm);
    }
  }
  if (temp == LS_W_U_SCALED_REG_DECODE) {

    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }

    goto LOAD_STORE_W_U_INSTS;

  } else if (temp == LS_W_U_SCALED_REG_PR_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }

    *reg_p = ls_address;

    goto LOAD_STORE_W_U_INSTS;
  }
  temp = OP_CODE & LS_W_U_SCALED_REG_PO_MASK;
  if (temp == LS_W_U_SCALED_REG_PO_DECODE) {
    ls_address = *reg_p;
    if (U_BIT) {
      *reg_p = *reg_p + shifter_operand;
    } else {
      *reg_p = *reg_p - shifter_operand;
    }

    if (IS_BIT_SET(OP_CODE, 21)) {
      goto LOAD_STORE_W_U_T_INSTS;
    }
    goto LOAD_STORE_W_U_INSTS;
  }

  goto UNDEFINED;
#undef OFFSET_12

LOAD_STORE_M:
#define reg_list shift_imm
#define set_bits rotate_imm
#define start_address shifter_operand
#define end_address ls_address
  reg_list = OP_CODE & 0xFFFF;
  set_bits = 0;
  reg_count = arm->mode * 16;
  reg_p = arm->reg_table[reg_count + RN_C];
  // counting number of 1 bits
  while (reg_list) {
    set_bits += reg_list & 1;
    reg_list >>= 1;
  }
  temp = OP_CODE & LS_M_MASK;
  if (temp == LS_M_IA_DECODE) {

    start_address = *reg_p;
    end_address = *reg_p + (set_bits * 4) - 4;
    if (IS_BIT_SET(OP_CODE, 21)) {
      *reg_p = end_address + 4;
    }
  } else if (temp == LS_M_IB_DECODE) {

    start_address = *reg_p + 4;
    end_address = *reg_p + (set_bits * 4);
    if (IS_BIT_SET(OP_CODE, 21)) {
      *reg_p = end_address;
    }

  } else if (temp == LS_M_DA_DECODE) {
    start_address = *reg_p - (set_bits * 4) + 4;
    end_address = *reg_p;

    if (IS_BIT_SET(OP_CODE, 21)) {
      *reg_p = start_address - 4;
    }

  } else if (temp == LS_M_DB_DECODE) {
    start_address = *reg_p - (set_bits * 4);
    end_address = *reg_p - 4;
    if (IS_BIT_SET(OP_CODE, 21)) {
      *reg_p = start_address;
    }
  }

  goto LOAD_STORE_M_INSTS;
#undef reg_list
#undef set_bits
#undef start_address
#undef end_address
DATA_PROCESS:

  cpu_cycle = 1;

  // shifter operand processing

#define INST_OPCODE ((OP_CODE >> 21) & 0xF)
  reg_count = arm->mode * 16;

  rn = *arm->reg_table[reg_count + RN_C];
  rd = RD_C;
  if (rd == R_15) {
    cpu_cycle += 2;
  }
  reg_p = arm->reg_table[reg_count + rd];
  s_bit = S_BIT;

  if ((OP_CODE & SHIFTER_IMM_MASK) == SHIFTER_IMM_DECODE) {
    rotate_imm = ROTATE_IMM;

    if (rotate_imm == 0) {
      shifter_operand = IMM_8;
      shifter_carry_out = IS_BIT_SET(arm->cpsr, CF_BIT);
    } else {
      rotate_imm *= 2;
      shifter_operand = ROTATE_RIGHT32((unsigned int)IMM_8, rotate_imm);
      shifter_carry_out = IS_BIT_SET(shifter_operand, 31);
    }
    goto *dp_inst_table[INST_OPCODE];
  }

  temp = OP_CODE & SHIFTER_REG_MASK;
  rm = *arm->reg_table[reg_count + RM_C];
  if (temp == SHIFTER_REG_DECODE) {
    shifter_operand = rm; // TODO - change to register
    shifter_carry_out = IS_BIT_SET(arm->cpsr, CF_BIT);
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_ROR_EXTEND_DECODE) {

    temp = (arm->cpsr << 2) & 0x80000000; // c flag << 31
    shifter_operand = temp | (rm >> 1);
    shifter_carry_out = rm & 1; // bit 0
    goto *dp_inst_table[INST_OPCODE];
  }

  temp = OP_CODE & SHIFTER_SHIFT_IMM_MASK;
  if (temp == SHIFTER_LSL_IMM_DECODE) {
    shift_imm = SHIFT_IMM;

    shifter_operand = rm << shift_imm;
    shifter_carry_out = GET_BIT(rm, (32 - shift_imm));
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_LSR_IMM_DECODE) {
    shift_imm = SHIFT_IMM;
    if (shift_imm == 0) {
      shifter_operand = 0;
      shifter_carry_out = GET_BIT(rm, 31);
    } else {
      shifter_operand = rm >> shift_imm;
      shifter_carry_out = GET_BIT(rm, (shift_imm - 1));
    }
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_ASR_IMM_DECODE) {
    shift_imm = SHIFT_IMM;
    if (shift_imm == 0) {

      shifter_carry_out = GET_BIT(rm, 31);
      shifter_operand = 0xFFFFFFFF * shifter_carry_out;

    } else {

      shifter_operand = ASR_32(rm, shift_imm);
      shifter_carry_out = GET_BIT(rm, (shift_imm - 1));
      // Todo check arithmetic shift right
      // shifter_operand = ASR_SIGN_32(rm, shift_imm, shifter_carry_out);
      // shifter_operand = (rm >> shift_imm) |
      //                   (shifter_carry_out * (0xFFFFFFFF << (32 -
      //                   shift_imm)));
    }
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_ROR_IMM_DECODE) {

    shift_imm = SHIFT_IMM;
    // shift_imm is not zero here
    shifter_operand = ROTATE_RIGHT32(rm, shift_imm);
    shifter_carry_out = GET_BIT(rm, (shift_imm - 1));
    goto *dp_inst_table[INST_OPCODE];
  }

  temp = OP_CODE & SHIFTER_SHIFT_REG_MASK;
  cpu_cycle += 1;

  if (RM_C == R_15) {
    rm += 4;
  }
  if (RN_C == R_15) {
    rn += 4;
  }
  rs = (*arm->reg_table[reg_count + RS_C]) &
       0xFF; // least significant byte of register rs
  if (temp == SHIFTER_LSL_REG_DECODE) {

    // instruction prefetching pc increase by 12
    if (rs == 0) {
      shifter_operand = rm;
      shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    } else if (rs < 32) {
      shifter_operand = rm << rs;
      shifter_carry_out = GET_BIT(rm, (32 - rs));
    } else if (rs == 32) {
      shifter_operand = 0;
      shifter_carry_out = rm & 1; // bit 0
    } else {
      shifter_operand = 0;
      shifter_carry_out = 0;
    }

    goto *dp_inst_table[INST_OPCODE];
  } else if (temp == SHIFTER_LSR_REG_DECODE) {

    if (rs == 0) {
      shifter_operand = rm;
      shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    } else if (rs < 32) {
      shifter_operand = rm >> rs;
      shifter_carry_out = GET_BIT(rm, (rs - 1));
    } else if (rs == 32) {
      shifter_operand = 0;
      shifter_carry_out = GET_BIT(rm, 31);
    } else {
      shifter_operand = 0;
      shifter_carry_out = 0;
    }

    goto *dp_inst_table[INST_OPCODE];
  } else if (temp == SHIFTER_ASR_REG_DECODE) {

    if (rs == 0) {
      shifter_operand = rm;
      shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    } else if (rs < 32) {
      shifter_operand = ASR_32(rm, rs);
      // shifter_operand =
      //     (rm >> rs) | (GET_BIT(rm, 31) * (0xFFFFFFFF << (32 - rs)));
      shifter_carry_out = GET_BIT(rm, (rs - 1));
    } else {
      shifter_carry_out = GET_BIT(rm, 31);
      shifter_operand = shifter_carry_out * 0xFFFFFFFF;
    }

    goto *dp_inst_table[INST_OPCODE];
  } else if (temp == SHIFTER_ROR_REG_DECODE) {

    temp = rs & 0x1F;
    if (rs == 0) {
      shifter_operand = rm;
      shifter_carry_out = IS_BIT_SET(arm->cpsr, CF_BIT);
    } else if (temp == 0) {
      shifter_operand = rm;
      shifter_carry_out = IS_BIT_SET(rm, 31);
    } else {
      shifter_operand = ROTATE_RIGHT32(rm, temp);
      shifter_carry_out = IS_BIT_SET(rm, (temp - 1));
    }
    goto *dp_inst_table[INST_OPCODE];
  }

  cpu_cycle = 1;
  goto END;
SWI:
  // TODO check implementation {!Its Wrong}
  SWI_INSTRUCTION(arm);
  cpu_cycle = 3;
  goto END;
CONTROL:

#define operand shifter_operand
  reg_count = arm->mode * 16;
  if ((OP_CODE & BX_MASK) == BX_DECODE) {
    rm = *arm->reg_table[reg_count + RM_C];
    // T bit
    arm->cpsr = SET_BIT(arm->cpsr, 5, (rm & 1));
    arm->state = rm & 1;
    arm->general_regs[R_15] = rm & 0xFFFFFFFE;
    arm->curr_instruction = arm->general_regs[R_15];
    cpu_cycle = 3;
    goto END;

  } else if ((OP_CODE & MRS_MASK) == MRS_DECODE) {
    reg_p = arm->reg_table[reg_count + RD_C];
    if (IS_BIT_SET(OP_CODE, 22)) {
      if (arm->mode > 1) {
        *reg_p = arm->spsr[arm->mode - 2];
      } else {
        // TODO unpredictable
        *reg_p = arm->cpsr;
      }
    } else {
      *reg_p = arm->cpsr;
    }
    write_instruction_log(arm, "MRS");
    goto END;

  } else if ((OP_CODE & MSR_IMM_MASK) == MSR_IMM_DECODE) {
    rotate_imm = ROTATE_IMM * 2;
    operand = ROTATE_RIGHT32((unsigned int)IMM_8, rotate_imm);

  } else if ((OP_CODE & MSR_REG_MASK) == MSR_REG_DECODE) {
    operand = *arm->reg_table[reg_count + RM_C];
  } else {
    // error undefined opcode
    goto UNDEFINED;
  }

  // bit mask constants for arm v4T
#define unalloc_mask 0x0FFFFF00
#define user_mask 0xF0000000
#define private_mask 0x0000000F
#define state_mask 0x00000020
#define byte_mask temp
#define mask shift_imm
#define fieldmask_bit0 16
#define fieldmask_bit1 17
#define fieldmask_bit2 18
#define fieldmask_bit3 19

  if ((operand & unalloc_mask) == 0) {
    byte_mask = IS_BIT_SET(OP_CODE, fieldmask_bit0) * 0x000000FF;
    byte_mask |= IS_BIT_SET(OP_CODE, fieldmask_bit1) * 0x0000FF00;
    byte_mask |= IS_BIT_SET(OP_CODE, fieldmask_bit2) * 0x00FF0000;
    byte_mask |= IS_BIT_SET(OP_CODE, fieldmask_bit3) * 0xFF000000;
  }

  if (IS_BIT_SET(OP_CODE, 22)) {
    if (arm->mode > 1) {
      mask = byte_mask & (user_mask | private_mask | state_mask);
      arm->spsr[arm->mode - 2] =
          (arm->spsr[arm->mode - 2] & (~mask)) | (operand & mask);
    }
    // unpredictable

  } else {
    if (arm->mode > 0) {
      // in privileged mode
      if ((operand & state_mask) == 0) {

        mask = byte_mask & (user_mask | private_mask);
      }
      // unpredictable
    } else {
      // user mode
      mask = byte_mask & user_mask;
    }
    arm->cpsr = (arm->cpsr & (~mask)) | (operand & mask);
    // set state, mode
    arm->state = IS_BIT_SET(arm->cpsr, 5);
    arm->mode = processor_modes[arm->cpsr & 0xF];
    if (arm->mode > 6) {
      printf("Not a valid processor mode\n");
      exit(1);
    }
  }
#ifdef DEBUG_ON
  write_instruction_log(arm, "MSR");
#endif

#undef operand
#undef unalloc_mask
#undef user_mask
#undef private_mask
#undef state_mask
#undef byte_mask
#undef mask
#undef fieldmask_bit0
#undef fieldmask_bit1
#undef fieldmask_bit2
#undef fieldmask_bit3
  goto END;
BRANCH_LINK:

  // TODO make sure sign extend is correct
  shifter_operand = OP_CODE & 0xFFFFFF;
  shifter_operand = shifter_operand | (IS_BIT_SET(shifter_operand, 23) *
                                       0x3F000000); // sign extend to 30 bits
  shifter_operand <<= 2;
  if (IS_BIT_SET(OP_CODE, 24)) {
    // LR = address of the instruction after the branch instruction
    *arm->reg_table[reg_count] = arm->curr_instruction;
  }
  arm->general_regs[R_15] += shifter_operand;
  arm->curr_instruction = arm->general_regs[R_15];
  write_instruction_log(arm, "B,BL");
  cpu_cycle = 3;
  goto END;
COPROCESSOR:
  goto END;
UNDEFINED:
  goto END;

UNCONDITIONAL:
  // for testing purposes only
  goto END;
  // unpredictable

AND_INST:
  *reg_p = rn & shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZC(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "AND");
#endif
  goto END;
EOR_INST:
  *reg_p = rn ^ shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZC(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "EOR");
#endif
  goto END;
SUB_INST:

  shifter_operand = (~shifter_operand); // two's compliment
  result = rn + ((uint64_t)shifter_operand + 1);
  *reg_p = result;
  shifter_operand += 1;
  DATA_PROCESS_RD_EQ_R15(arm);
  if (s_bit) {
    DATA_PROCESS_NZCV();
    // printf("cpsr - %d\n", arm->cpsr);
  }

#ifdef DEBUG_ON
  write_instruction_log(arm, "SUB");
#endif

  // write_instruction_log(arm, "sub");
  goto END;
RSB_INST:
  rn = (~rn);
  result = ((uint64_t)rn + 1) + shifter_operand;
  rn += 1;
  *reg_p = result;

  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZCV(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "RSB");
#endif

  goto END;

ADD_INST:
  result = rn + (uint64_t)shifter_operand;
  *reg_p = result;

  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZCV(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD");
#endif
  // write_instruction_log(arm, "add");
  goto END;
ADC_INST:

  result = rn + (uint64_t)shifter_operand + IS_BIT_SET(arm->cpsr, CF_BIT);
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZCV(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADC");
#endif
  goto END;

SBC_INST:
  shifter_operand = (~shifter_operand);
  result =
      rn + ((uint64_t)shifter_operand + 1) - IS_BIT_NOT_SET(arm->cpsr, CF_BIT);
  *reg_p = result;
  shifter_operand += 1;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZCV(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "SBC");
#endif
  goto END;
RSC_INST:
  rn = (~rn);
  result =
      shifter_operand + ((uint64_t)rn + 1) - IS_BIT_NOT_SET(arm->cpsr, CF_BIT);
  *reg_p = result;
  rn += 1;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZCV(); }

#ifdef DEBUG_ON
  write_instruction_log(arm, "RSC");
#endif
  goto END;
TST_INST:
  result = rn & shifter_operand;
  COMPARE_INSTS_NZC();
#ifdef DEBUG_ON
  write_instruction_log(arm, "TST");
#endif
  goto END;
TEQ_INST:
  result = rn ^ shifter_operand;
  COMPARE_INSTS_NZC();

#ifdef DEBUG_ON
  write_instruction_log(arm, "TEQ");
#endif
  goto END;
CMP_INST:
  shifter_operand = (~shifter_operand);
  result = rn + ((uint64_t)shifter_operand + 1);
  shifter_operand += 1;
  COMPARE_INSTS_NZCV();
#ifdef DEBUG_ON
  write_instruction_log(arm, "CMP");
#endif
  goto END;
CMN_INST:
  result = rn + (uint64_t)shifter_operand;
  COMPARE_INSTS_NZCV();
#ifdef DEBUG_ON
  write_instruction_log(arm, "CMN");
#endif
  goto END;
ORR_INST:
  *reg_p = rn | shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZC(); }

#ifdef DEBUG_ON
  write_instruction_log(arm, "OR");
#endif

  goto END;
MOV_INST:
  *reg_p = shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZC(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "MOV");
#endif

  // write_instruction_log(arm, "mov");
  goto END;
BIC_INST:
  *reg_p = rn & (~shifter_operand);
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZC(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "BIC");
#endif
  goto END;
MVN_INST:
  *reg_p = ~shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) if (s_bit) { DATA_PROCESS_NZC(); }
#ifdef DEBUG_ON
  write_instruction_log(arm, "MVN");
#endif

  goto END;

LOAD_STORE_H_D_S_INSTS:

  temp = OP_CODE & LS_H_D_S_INST_MASK;
  rd = RD_C;
  reg_count += rd;
  if (temp == LDRH_DECODE) {

    // *arm->reg_table[reg_count] = arm_read(ls_address) & 0xFFFF;
    ARM_READ(ls_address, *arm->reg_table[reg_count], mem_read16);
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == LDRSB_DECODE) {
    // shifter_operand = arm_read(ls_address) & 0xFF;
    ARM_READ(ls_address, shifter_operand, mem_read8);
    shifter_operand = SIGN_EXTEND(shifter_operand, 7);

    *arm->reg_table[reg_count] = shifter_operand;
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == LDRSH_DECODE) {
    // shifter_operand = arm_read(ls_address) & 0xFFFF;
    ARM_READ(ls_address, shifter_operand, mem_read16);
    shifter_operand = SIGN_EXTEND(shifter_operand, 15);

    *arm->reg_table[reg_count] = shifter_operand;
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == STRH_DECODE) {
    // TODO implement it
    ARM_WRITE(ls_address, *arm->reg_table[reg_count], mem_write16);
    cpu_cycle = 2;
  }
  goto END;

LOAD_STORE_W_U_T_INSTS:

  // write_decoder_log(arm, "LOAD_STORE_W_U_T_INSTS");
  // Instructions can be LDRBT, LDRT, STRBT, STRT
  temp = OP_CODE & 0x1700000;
  rd = RD_C;
  reg_p = arm->reg_table[reg_count + rd];
  if (temp == LDRBT_DECODE) {
    // *reg_p = arm_read(ls_address);
    ARM_READ(ls_address, *reg_p, mem_read8);
    *arm->reg_table[reg_count + RN_C] = ls_address;
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == STRBT_DECODE) {
    ARM_WRITE(ls_address, *reg_p, mem_write8);
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == LDRT_DECODE) {

    // *reg_p = arm_read(ls_address);
    ARM_READ(ls_address, *reg_p, mem_read32);
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == STRT_DECODE) {
    ARM_WRITE(ls_address, *reg_p, mem_write32);
    cpu_cycle = 2;
  }
  goto END;

LOAD_STORE_W_U_INSTS:

  // write_decoder_log(arm, "LoadStore_W_U Instruction");
  // Instructions can be LDR, LDRB, STR, STRB
  temp = OP_CODE & 0x500000;
  rd = RD_C;
  reg_p = arm->reg_table[reg_count + rd];
  if (temp == LDR_DECODE) {
    // result = arm_read(ls_address);
    ARM_READ(ls_address, result, mem_read32);
    if (rd == R_15) {
      *reg_p = result & 0xFFFFFFFC;
      arm->curr_instruction = *reg_p;
    } else {
      *reg_p = result;
    }
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == STR_DECODE) {
    ARM_WRITE(ls_address, *reg_p, mem_write32); // word write
    cpu_cycle = 2;

  } else if (temp == LDRB_DECODE) {
    // *reg_p = arm_read(ls_address); // unsigned byte memory access
    ARM_READ(ls_address, *reg_p, mem_read8);
    cpu_cycle = (rd == R_15) ? 5 : 3;

  } else if (temp == STRB_DECODE) {
    ARM_WRITE(ls_address, *reg_p, mem_write8); // byte write
    cpu_cycle = 2;
  }

  cpu_cycle = (rd == R_15) ? 5 : 3;
  goto END;

LOAD_STORE_M_INSTS:
#define start_address shifter_operand
#define end_address ls_address
#define reg_list shift_imm
#define i shifter_carry_out
  reg_list = OP_CODE & 0xFFFF;
  reg_count = arm->mode * 16;

  if ((OP_CODE & LDM1_MASK) == LDM1_DECODE) {
    cpu_cycle = 0;
    for (i = 0; i < 15; i++) {
      if (reg_list & 1) {
        cpu_cycle += 1;
        // *arm->reg_table[reg_count + i] = arm_read(start_address);
        ARM_READ(start_address, *arm->reg_table[reg_count + i], mem_read32);
        start_address += 4;
      }
      reg_list >>= 1;
    }

    cpu_cycle += 2;

    if (reg_list & 1) {
      // check for pc
      // arm->general_regs[R_15] = arm_read(start_address) & 0xFFFFFFFC;

      cpu_cycle += 3;

      ARM_READ(start_address, arm->general_regs[R_15], mem_read32);
      arm->curr_instruction = arm->general_regs[R_15];
      start_address += 4;
    }
    assert(end_address == start_address - 4);

  } else if ((OP_CODE & LDM2_MASK) == LDM2_DECODE) {
    cpu_cycle = 0;
    for (i = 0; i < 15; i++) {
      if (reg_list & 1) {
        cpu_cycle += 1;
        // arm->general_regs[i] = arm_read(start_address);
        ARM_READ(start_address, arm->general_regs[i], mem_read32);
        start_address += 4;
      }
      reg_list >>= 1;
    }
    cpu_cycle += 2;
    assert(end_address == start_address - 4);

  } else if ((OP_CODE & LDM3_MASK) == LDM3_DECODE) {
    cpu_cycle = 1;
    for (i = 0; i < 15; i++) {
      if (reg_list & 1) {
        cpu_cycle += 1;
        // *arm->reg_table[reg_count + i] = arm_read(start_address);
        ARM_READ(start_address, *arm->reg_table[reg_count + i], mem_read32);
        start_address += 4;
      }
      reg_list >>= 1;
    }
    if (arm->mode > 1) {
      arm->cpsr = arm->spsr[arm->mode - 2];
    }
    // arm->general_regs[R_15] = arm_read(start_address);
    ARM_READ(start_address, arm->general_regs[R_15], mem_read32);
    arm->curr_instruction = arm->general_regs[R_15];
    start_address += 4;
    assert(end_address == start_address - 4);
    cpu_cycle += 4;
  } else if ((OP_CODE & STM1_MASK) == STM1_DECODE) {
    cpu_cycle = 1;
    for (i = 0; i < 16; i++) {
      if (reg_list & 1) {
        cpu_cycle += 1;
        ARM_WRITE(start_address, *arm->reg_table[reg_count + i], mem_write32);
        start_address += 4;
      }
      reg_list >>= 1;
    }
    assert(end_address == start_address - 4);
  } else if ((OP_CODE & STM2_MASK) == STM2_DECODE) {
    cpu_cycle = 1;
    for (i = 0; i < 16; i++) {
      if (reg_list & 1) {
        cpu_cycle += 1;
        ARM_WRITE(start_address, arm->general_regs[i], mem_write32);
        start_address += 4;
      }
      reg_list >>= 1;
    }
    assert(end_address == start_address - 4);
  }
  goto END;

MUL_INST:
  *reg_p = rm * rs;
  if (s_bit) {
    MUL_NZ(arm);
  }
  cpu_cycle += 1;
  goto END;
MLA_INST:
  *reg_p = (rm * rs) + *arm->reg_table[rn];
  if (s_bit) {
    MUL_NZ(arm);
  }
  cpu_cycle += 2;
  goto END;
UMULL_INST:
  result = (uint64_t)rm * rs;
  *reg_p = result >> 32;        // rdhi
  *arm->reg_table[rn] = result; // rdlow
  if (s_bit) {
    USMULL_NZ(arm);
  }
  cpu_cycle += 2;
  goto END;

UMLAL_INST:
  // RdLo = (Rm * Rs)[31:0] + RdLo
  // /* Unsigned multiplication */
  // RdHi = (Rm * Rs)[63:32] + RdHi + CarryFrom((Rm * Rs)[31:0] + RdLo)
  // if S == 1 then
  // N Flag = RdHi[31]
  // Z Flag = if (RdHi == 0) and (RdLo == 0) then 1 else 0
  // C Flag = unaffected
  // /* See "C and V flags" note */
  // V Flag = unaffected
  // /* See "C and V flags" note */
  result = (uint64_t)rm * rs;
  *reg_p += (result >> 32);
  result = (result & 0xFFFFFFFF) + *arm->reg_table[rn];
  *reg_p += GET_BIT(result, 32);
  *arm->reg_table[rn] = result;
  if (s_bit) {
    USMULL_NZ(arm);
  }
  cpu_cycle += 3;
  goto END;
SMULL_INST:
  // TODO: make sure it is correct
  result = (int32_t)rm * (int32_t)rs;
  *reg_p = (result >> 32);
  *arm->reg_table[rn] = result;
  if (s_bit) {
    USMULL_NZ(arm);
  }
  cpu_cycle += 2;
  goto END;
SMLAL_INST:
  result = (int32_t)rm * (int32_t)rs;
  *reg_p += (result >> 32); // rdHi
  result = (result & 0xFFFFFFFF) + *arm->reg_table[rn];
  *reg_p += GET_BIT(result, 32);
  *arm->reg_table[rn] = result;
  if (s_bit) {
    USMULL_NZ(arm);
  }
  cpu_cycle += 3;
  goto END;

SWP_INST:
  reg_count = arm->mode * 16;
  rn = *arm->reg_table[reg_count + RN_C];
  rm = *arm->reg_table[reg_count + RM_C];
  MEM_READ(rn, shifter_operand, mem_read32);
  MEM_WRITE(rn, rm, mem_write32);
  *arm->reg_table[reg_count + RD_C] = shifter_operand;
  cpu_cycle = 4;
  goto END;

SWPB_INST:
  reg_count = arm->mode * 16;
  rn = *arm->reg_table[reg_count + RN_C];
  rm = *arm->reg_table[reg_count + RM_C];
  MEM_READ(rn, shifter_operand, mem_read8);
  MEM_WRITE(rn, rm, mem_write8);
  *arm->reg_table[reg_count + RD_C] = shifter_operand;
  cpu_cycle = 4;

END:
  return cpu_cycle;
}
