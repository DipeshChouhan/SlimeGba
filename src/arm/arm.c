/* TODO - Check decoder code */
// TODO - test rotate right and shifter operand value   {DONE}
// TODO seprate flag setting and cpsr = spsr in data processing instructions
// {DONE}
// TODO CPSR = SPSR code is incorrect Check again {DONE}
// TODO optimize overflow flag setting  {DONE}
// TODO choose registers based on mode in instructions  !{DONE - NOT TESTED}
// TODO remove checking condition passed !{CHECK CORRECT IMPL}
//
// TODO implement memory read and write change in all load and store
// !{IMPORTANT}
#include "arm.h"
#include "disassembler.h"
#include "arm_inst_decode.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define DEBUG_ON

#define NF_BIT 31
#define ZF_BIT 30
#define CF_BIT 29
#define VF_BIT 28
#define R_15 15
#define U_BIT (OP_CODE & 0x800000)

#define RN_C ((OP_CODE >> 16) & 0xF)
#define RD_C ((OP_CODE >> 12) & 0xF)
#define RM_C (OP_CODE & 0xF)
#define RS_C ((OP_CODE >> 8) & 0xF)
#define SHIFT_IMM ((OP_CODE >> 7) & 0x1F)

#define DATA_PROCESS_NZCV(_arm)                                                \
  temp = _arm->cpsr & 0xFFFFFFF;                                               \
  temp |= (result & 0x80000000);                                               \
  temp |= ((*reg_p == 0) << 30);                                               \
  temp |= ((result & 0x100000000) >> 3);                                       \
  temp |= ((((rn & 0x80000000) == (shifter_operand & 0x80000000)) &&           \
            ((rn & 0x80000000) != (result & 0x80000000)))                      \
           << 28);

#define DATA_PROCESS_NZC(_arm)                                                 \
  temp = _arm->cpsr & 0x1FFFFFFF;                                              \
  temp |= (result & 0x80000000);                                               \
  temp |= ((*reg_p == 0) << 30);                                               \
  temp |= (shifter_carry_out << 29);

#define MUL_NZ(_arm)                                                           \
  temp = _arm->cpsr & 0x3FFFFFFF;                                              \
  temp |= (result & 0x80000000);                                               \
  temp |= ((*reg_p == 0) << 30);

#define DATA_PROCESS_RD_EQ_R15(_arm)                                           \
  if (s_bit && rd == R_15) {                                                   \
    if (_arm->mode > 1) {                                                      \
      _arm->cpsr = _arm->spsr[_arm->mode - 2];                                 \
    }                                                                          \
  }

#define GET_BIT(_op, _bit) ((_op >> _bit) & 1)

#define SET_BIT(_op, _bit, _to) ((_op & (~(1 << _bit))) | (_to << _bit))

#define IS_BIT_SET(_op, _bit) ((_op & (1 << _bit)) == (1 << _bit))
#define OP_CODE arm->data_bus

// not considering _ror is zero
#define ROTATE_RIGHT32(_op, _ror) ((_op >> _ror) | (_op << (32 - _ror)))

// TODO Implement this function
uint32_t arm_read(uint32_t addr) { return 0; }

void arm_write(uint32_t addr, uint32_t value);

void gen_reg_table(Arm *arm) {

  for (int mode = 0; mode < 7; mode++) {
    for (int reg = 0; reg < 16; reg++) {
      if (mode < 2) {
        // user mode and system mode
        arm->reg_table[reg] = &arm->general_regs[reg];
        continue;
      }
      if (mode == FIQ) {
        // fiq mode
        if (reg >= 8 && reg <= 14) {
          arm->reg_table[reg] = &arm->fiq_regs[reg - 8];
          continue;
        }
        arm->reg_table[reg] = &arm->general_regs[reg];
        continue;
      }

      if (reg == 13 || reg == 14) {
        if (mode == IRQ) {
          arm->reg_table[reg] = &arm->irq_regs[reg - 13];

        } else if (mode == SVC) {
          arm->reg_table[reg] = &arm->svc_regs[reg - 13];

        } else if (mode == ABT) {
          arm->reg_table[reg] = &arm->abt_regs[reg - 13];

        } else {
          // und
          arm->reg_table[reg] = &arm->und_regs[reg - 13];
        }
        continue;
      }

      arm->reg_table[reg] = &arm->general_regs[reg];
    }
  }
}

void init_arm(Arm *arm) {

  int index = 0;

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
  arm->spsr_fiq = 0;
  arm->spsr_abt = 0;
  arm->spsr_irq = 0;
  arm->spsr_svc = 0;
  arm->spsr_und = 0;
  arm->state = 0; // default arm state
  gen_reg_table(arm);

  // all global test goes here
#ifdef DEBUG_ON
  uint32_t test_val = ROTATE_RIGHT32(0x3F, 0xE * 2);
  if (test_val != 0x3F0) {
    printf("ROTATE_RIGHT32 Failed #1\n");
    exit(1);
  }
  test_val = ROTATE_RIGHT32(0xFC, 0xF * 2);
  if (test_val != 0x3F0) {

    printf("ROTATE_RIGHT32 Failed #2\n");
    exit(1);
  }
#endif
}

int arm_exec(Arm *arm) {

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
  int cond_passed = 0;
  uint32_t shift = 0;

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
  // fetch() opcode in data bus
  // do decode
  // goto DECODE;

  goto *cond_field_table[OP_CODE >> 28];

// check for conditions and set execute_instruction to 1
CHECK_EQ:
CHECK_NE:
CHECK_CS_HS:
CHECK_CC_LO:
CHECK_MI:
CHECK_PL:
CHECK_VS:
CHECK_VC:
CHECK_HI:
CHECK_LS:
CHECK_GE:
CHECK_LT:
CHECK_GT:
CHECK_LE:
CHECK_AL:

DECODE:
  if ((OP_CODE & MULTIPLY_MASK) == MULTIPLY_DECODE) {
    // multiply instructions

    temp = (OP_CODE >> 21) & 0x7;
    reg_count = arm->mode * 16;
    reg_p = arm->reg_table[reg_count + RN_C];

    rs = *arm->reg_table[reg_count + RS_C];
    rm = *arm->reg_table[reg_count + RM_C];

    write_instruction_log(arm, "multiply");
    goto *mul_inst_table[temp];
  } else if ((OP_CODE & LOAD_STORE_H_D_S_MASK) == LOAD_STORE_H_D_S_DECODE) {
    // Load and store halfword or doubleword, and load signed byte instructions
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
  immedH = (OP_CODE & 0xF00) >> 8;
  reg_count = arm->mode * 16;
  rn = RN_C;
  reg_p = arm->reg_table[reg_count+ rn];
  rm = *arm->reg_table[reg_count + RM_C];
  temp = OP_CODE & LS_H_D_S_MASK;
  offset_8 = ((immedH << 4) | immedL) & 0xFF;
  if (temp == LS_H_D_S_REG_DECODE) {

    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }

  } else if (temp == LS_H_D_S_IMM_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + offset_8;
    } else {
      ls_address = *reg_p - offset_8;
    }

  } else if (temp == LS_H_D_S_IMM_PR_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + offset_8;
    } else {
      ls_address = *reg_p - offset_8;
    }
    if (cond_passed) {
      *reg_p = ls_address;
    }

  } else if (temp == LS_H_D_S_REG_PR_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }
    if (cond_passed) {
      *reg_p = ls_address;
    }

  } else if (temp == LS_H_D_S_REG_PO_DECODE) {
    ls_address = *reg_p;
    if (cond_passed) {
      if (U_BIT) {
        *reg_p = *reg_p + rm;
      } else {
        *reg_p = *reg_p - rm;
      }
    }
  } else if (temp == LS_H_D_S_IMM_PO_DECODE) {
    ls_address = *reg_p;
    if (cond_passed) {
      if (U_BIT) {
        *reg_p = *reg_p + offset_8;
      } else {
        *reg_p = *reg_p - offset_8;
      }
    }
  }
  write_instruction_log(arm, "load_store_h_d_s");
  goto END;
LOAD_STORE_W_U:
#define OFFSET_12 (OP_CODE & 0xFFF)

  rn = RN_C;
  reg_count = arm->mode * 16;
  // reg_count = RN_C;
  // rn = arm->general_regs[reg_count];
  reg_p = arm->reg_table[reg_count + rn];
  shifter_operand = OFFSET_12;
  temp = OP_CODE & LS_W_U_IMM_MASK;
  if (temp == LS_W_U_IMM_DECODE) {

    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }

  } else if (temp == LS_W_U_IMM_PR_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }

    if (cond_passed) {
      arm->general_regs[reg_count] = ls_address;
    }
  }
  temp = OP_CODE & LS_W_U_IMM_PO_MASK;
  if (temp == LS_W_U_IMM_PO_DECODE) {
    ls_address = *reg_p;
    if (cond_passed) {
      if (U_BIT) {
        *reg_p = *reg_p + shifter_operand;
      } else {
        *reg_p = *reg_p - shifter_operand;
      }
    }
  }

  rm = *arm->reg_table[reg_count + RM_C];
  temp = OP_CODE & LS_W_U_REG_MASK;
  if (temp == LS_W_U_REG_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }

  } else if (temp == LS_W_U_REG_PR_DECODE) {

    if (U_BIT) {
      ls_address = *reg_p + rm;
    } else {
      ls_address = *reg_p - rm;
    }

    if (cond_passed) {
      *reg_p = ls_address;
    }
  }
  temp = OP_CODE & LS_W_U_REG_PO_MASK;
  if (temp == LS_W_U_REG_PO_DECODE) {

    ls_address = *reg_p;
    if (cond_passed) {
      if (U_BIT) {
        *reg_p = *reg_p + rm;
      } else {
        *reg_p = *reg_p - rm;
      }
    }
  }

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
    if (shift_imm == 0) {
      s_bit = GET_BIT(rm, 31);
      if (s_bit)
        shifter_operand = 0xFFFFFFFF;
      else
        shifter_operand = 0;
    } else {

      shifter_operand =
          (rm >> shift_imm) | (s_bit * (0xFFFFFFFF << (32 - shift_imm)));
    }
    break;

  default:
    if (shift_imm == 0) {
      s_bit = GET_BIT(arm->cpsr, CF_BIT);
      shifter_operand = (s_bit << 31) | (rm >> 1);
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

  } else if (temp == LS_W_U_SCALED_REG_PR_DECODE) {
    if (U_BIT) {
      ls_address = *reg_p + shifter_operand;
    } else {
      ls_address = *reg_p - shifter_operand;
    }

    if (cond_passed) {
      *reg_p = ls_address;
    }
  }
  temp = OP_CODE & LS_W_U_SCALED_REG_PO_MASK;
  if (temp == LS_W_U_SCALED_REG_PO_DECODE) {
    if (cond_passed) {
      if (U_BIT) {
        *reg_p = *reg_p + shifter_operand;
      } else {
        *reg_p = *reg_p - shifter_operand;
      }
    }
  }
  write_instruction_log(arm, "load_store_w_u");
  goto END;
LOAD_STORE_M:
  temp = OP_CODE & LS_M_MASK;
  if (temp == LS_M_IA_DECODE) {

  } else if (temp == LS_M_IB_DECODE) {

  } else if (temp == LS_M_DA_DECODE) {

  } else if (temp == LS_M_DB_DECODE) {
  }
  write_instruction_log(arm, "load_store_m");
  goto END;
DATA_PROCESS:

  // shifter operand processing

#define ROTATE_IMM ((OP_CODE >> 8) & 0xF)
#define IMM_8 (OP_CODE & 0xFF)
#define S_BIT (OP_CODE & 0x100000)
#define INST_OPCODE ((OP_CODE >> 21) & 0xF)
  reg_count = arm->mode * 16;
  rn = *arm->reg_table[reg_count + RN_C];
  rd = RD_C;
  reg_p = arm->reg_table[reg_count + rd];
  s_bit = S_BIT;

  if ((OP_CODE & SHIFTER_IMM_MASK) == SHIFTER_IMM_DECODE) {
    rotate_imm = ROTATE_IMM;

    if (rotate_imm == 0) {
      shifter_operand = IMM_8;
      shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    } else {
      rotate_imm *= 2;
      shifter_operand = ROTATE_RIGHT32(IMM_8, rotate_imm);
      shifter_carry_out = GET_BIT(shifter_operand, 31);
    }
    goto *dp_inst_table[INST_OPCODE];
  }

  temp = OP_CODE & SHIFTER_REG_MASK;
  rm = *arm->reg_table[reg_count + RM_C];
  if (temp == SHIFTER_REG_DECODE) {
    shifter_operand = rm; // TODO - change to register
    shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_ROR_EXTEND_DECODE) {

    temp = GET_BIT(arm->cpsr, CF_BIT) << 31;
    shifter_operand = temp | (rm >> 1);
    shifter_carry_out = rm & 1; // bit 0
    goto *dp_inst_table[INST_OPCODE];
  }

  temp = OP_CODE & SHIFTER_SHIFT_IMM_MASK;
  shift_imm = SHIFT_IMM;
  if (temp == SHIFTER_LSL_IMM_DECODE) {

    shifter_operand = rm << shift_imm;
    shifter_carry_out = GET_BIT(rm, (32 - shift_imm));
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_LSR_IMM_DECODE) {
    if (shift_imm == 0) {
      shifter_operand = 0;
      shifter_carry_out = GET_BIT(rm, 31);
    } else {
      shifter_operand = rm >> shift_imm;
      shifter_carry_out = GET_BIT(rm, (shift_imm - 1));
    }
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_ASR_IMM_DECODE) {
    if (shift_imm == 0) {

      shifter_carry_out = GET_BIT(rm, 31);
      shifter_operand = 0xFFFFFFFF * shifter_carry_out;

    } else {

      shifter_carry_out = GET_BIT(rm, (shift_imm - 1));
      // Todo check arithmetic shift right
      shifter_operand = (rm >> shift_imm) |
                        (shifter_carry_out * (0xFFFFFFFF << (32 - shift_imm)));
    }
    goto *dp_inst_table[INST_OPCODE];

  } else if (temp == SHIFTER_ROR_IMM_DECODE) {

    shifter_operand = ROTATE_RIGHT32(rm, shift_imm);
    shifter_carry_out = GET_BIT(rm, (shift_imm - 1));
    goto *dp_inst_table[INST_OPCODE];
  }

  temp = OP_CODE & SHIFTER_SHIFT_REG_MASK;

  rs = (*arm->reg_table[reg_count + RS_C]) & 0xFF; // least significant byte of register rs
  if (temp == SHIFTER_LSL_REG_DECODE) {

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
      shifter_operand =
          (rm >> rs) | (GET_BIT(rm, 31) * (0xFFFFFFFF << (32 - rs)));
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
      shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    } else if (temp == 0) {
      shifter_operand = rm;
      shifter_carry_out = GET_BIT(rm, 31);
    } else {
      shifter_operand = ROTATE_RIGHT32(rm, temp);
      shifter_carry_out = GET_BIT(rm, (temp - 1));
    }
    goto *dp_inst_table[INST_OPCODE];
  }

  // write_instruction_log(arm, "data process");
  goto END;
SWI:
  write_instruction_log(arm, "swi");
  goto END;
CONTROL:
  write_instruction_log(arm, "control");
  goto END;
BRANCH_LINK:
  write_instruction_log(arm, "branch_link");
  goto END;
COPROCESSOR:
  write_instruction_log(arm, "coprocessor");
  goto END;
UNDEFINED:
  write_instruction_log(arm, "undefined");
  goto END;

UNCONDITIONAL:
  printf("UNCONDITIONAL\n");
  goto END;
  // unpredictable

AND_INST:
  result = rn & shifter_operand;
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZC(arm); }
  goto END;
EOR_INST:
  result = rn ^ shifter_operand;
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZC(arm); }
  goto END;
SUB_INST:

  shifter_operand = (~shifter_operand) + 1; // two's compliment
  result = rn + shifter_operand;
  *reg_p = result;

  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZCV(arm); }

  write_instruction_log(arm, "sub");
  goto END;
RSB_INST:
  rn = (~rn) + 1;
  result = rn + shifter_operand;
  *reg_p = result;

  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZCV(arm); }
  goto END;

ADD_INST:
  result = rn + shifter_operand;
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZCV(arm); }
  write_instruction_log(arm, "add");
  goto END;
ADC_INST:
  result = rn + shifter_operand + GET_BIT(arm->cpsr, CF_BIT);
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZCV(arm); }
  goto END;

SBC_INST:
  shifter_operand = (~shifter_operand) + GET_BIT(arm->cpsr, CF_BIT);
  result = rn + shifter_operand;
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZCV(arm); }
  goto END;
RSC_INST:
  rn = (~rn) + GET_BIT(arm->cpsr, CF_BIT);
  result = shifter_operand + rn;
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZCV(arm); }
  goto END;
TST_INST:
  result = rn & shifter_operand;
  *reg_p = result;
  DATA_PROCESS_NZC(arm);

  goto END;
TEQ_INST:
  result = rn ^ shifter_operand;
  *reg_p = result;
  DATA_PROCESS_NZC(arm);
  goto END;
CMP_INST:
  shifter_operand = (~shifter_operand) + 1;
  result = rn + shifter_operand;
  *reg_p = result;
  DATA_PROCESS_NZCV(arm);
  goto END;
CMN_INST:
  result = rn + shifter_operand;
  *reg_p = result;
  DATA_PROCESS_NZCV(arm);
  goto END;
ORR_INST:
  result = rn | shifter_operand;
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZC(arm); }

  goto END;
MOV_INST:
  *reg_p = shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) {

    temp = arm->cpsr & 0x1FFFFFFF;
    temp |= (arm->general_regs[rd] & 0x80000000);
    temp |= ((arm->general_regs[rd] == 0) << 30);
    temp |= (shifter_carry_out << 29);
  }

  write_instruction_log(arm, "mov");
  goto END;
BIC_INST:
  result = rn & (~shifter_operand);
  *reg_p = result;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) { DATA_PROCESS_NZC(arm); }
  goto END;
MVN_INST:
  *reg_p = shifter_operand;
  DATA_PROCESS_RD_EQ_R15(arm) else if (s_bit) {
    temp = arm->cpsr & 0x1FFFFFFF;
    temp |= (arm->general_regs[rd] & 0x80000000);
    temp |= ((arm->general_regs[rd] == 0) << 30);
    temp |= (shifter_carry_out << 29);
  }

LOAD_STORE_W_U_T_INSTS:

  // Instructions can be LDRBT, LDRT, STRBT, STRT
  temp = OP_CODE & 0x1700000;
  reg_p = arm->reg_table[reg_count + RD_C];
  if (temp == LDRBT_DECODE) {
    *reg_p = arm_read(ls_address);
    *arm->reg_table[reg_count + RN_C] = ls_address;

  } else if (temp == STRBT_DECODE) {
    arm_write(ls_address, (*reg_p) & 0xFF);

  } else if (temp == LDRT_DECODE) {

    *reg_p = arm_read(ls_address);

  } else if (temp == STRT_DECODE) {
    arm_write(ls_address, *reg_p);
  }

LOAD_STORE_W_U_INSTS:

  // Instructions can be LDR, LDRB, STR, STRB
  temp = OP_CODE & 0x500000;
  rd = RD_C;
  reg_p = arm->reg_table[reg_count + rd];
  if (temp == LDR_DECODE) {
    result = arm_read(ls_address);
    if (rd == 15) {
      *reg_p = result & 0xFFFFFFFC;
    } else {
      *reg_p = result;
    }

  } else if (temp == STR_DECODE) {
    arm_write(ls_address, *reg_p); // word write

  } else if (temp == LDRB_DECODE) {
    *reg_p = arm_read(ls_address); // unsigned byte memory access

  } else if (temp == STRB_DECODE) {
    arm_write(ls_address, (*reg_p) & 0xFF); // byte write
  }

MUL_INST:
  *reg_p = rm * rs;
MLA_INST:
  rn = *arm->reg_table[reg_count + RD_C];
  *reg_p = (rm * rs) + rn;
UMULL_INST:

UMLAL_INST:
SMULL_INST:
SMLAL_INST:

END:
  return 0;
}
