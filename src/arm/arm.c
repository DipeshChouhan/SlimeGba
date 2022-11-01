/* TODO - Check decoder code */
// TODO - test rotate right and shifter operand value
#include "arm.h"
#include "disassembler.h"
#include "inst_decode.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define DEBUG_ON

#define NF_BIT 31
#define ZF_BIT 30
#define CF_BIT 29
#define VF_BIT 28

#define GET_BIT(_op, _bit) ((_op >> _bit) & 1)
#define OP_CODE arm->data_bus

// not considering _ror is zero
#define ROTATE_RIGHT32(_op, _ror) ((_op >> _ror) | (_op << (32 - _ror)))

void init_arm(Arm *arm) {

  int index = 0;

  while(index < 16) {
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

  static void *dp_inst_table[] = {
      &&AND_INST, &&EOR_INST, &&SUB_INST, &&RSB_INST, &&ADD_INST, &&ADC_INST,
      &&SBC_INST, &&RSC_INST, &&TST_INST, &&TEQ_INST, &&CMP_INST, &&CMN_INST,
      &&ORR_INST, &&MOV_INST, &&BIC_INST, &&MVN_INST};
  //                                 static void *mult_inst_table[] = {
  //     &&DO_UND_MULTIPLY, &&DO_MUL,   &&DO_UND_MULTIPLY, &&DO_UND_MULTIPLY,
  //     &&DO_UMULL,        &&DO_UMLAL, &&DO_SMULL,        &&DO_SMLAL,
  //
  //
  // };
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
    goto MULTIPLY;

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

MULTIPLY:
  write_instruction_log(arm, "multiply");
  goto END;
LOAD_STORE_H_D_S:
  write_instruction_log(arm, "load_store_h_d_s");
  goto END;
LOAD_STORE_W_U:
  write_instruction_log(arm, "load_store_w_u");
  goto END;
LOAD_STORE_M:
  write_instruction_log(arm, "load_store_m");
  goto END;
DATA_PROCESS:

  // shifter operand processing

#define ROTATE_IMM ((OP_CODE >> 8) & 0xF)
#define IMM_8 (OP_CODE & 0xFF)
#define SHIFT_IMM ((OP_CODE & 7) & 0x1F)
#define RN ((OP_CODE >> 16) & 0xF)
#define RD ((OP_CODE >> 12) & 0xF)
#define RM (OP_CODE & 0xF)
#define RS ((OP_CODE >> 8) & 0xF)

  if ((OP_CODE & SHIFTER_IMM_MASK) == SHIFTER_IMM_DECODE) {
    // shifter_operand = immed_8 Rotate_Right (rotate_imm * 2)
    // if rotate_imm == 0 then
    // shifter_carry_out = C flag
    // else /* rotate_imm != 0 */
    // shifter_carry_out = shifter_operand[31]
    // immediate
    temp = ROTATE_IMM;

    if (temp == 0) {
      shifter_operand = IMM_8;
      shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);
    } else {
      temp *= 2;
      shifter_operand = ROTATE_RIGHT32(IMM_8, temp);
      shifter_carry_out = GET_BIT(shifter_operand, 31);
    }
  }

  temp = OP_CODE & SHIFTER_REG_MASK;
  if (temp == SHIFTER_REG_DECODE) {
    // shifter_operand = Rm
    // shifter_carry_out = C Flag
    shifter_operand = RM; // TODO - change to register
    shifter_carry_out = GET_BIT(arm->cpsr, CF_BIT);

  } else if (temp == SHIFTER_ROR_EXTEND_DECODE) {
  }

  temp = OP_CODE & SHIFTER_SHIFT_IMM_MASK;
  if (temp == SHIFTER_LSL_IMM_DECODE) {
    // printf("LSL imm\n");

    // The carry-out from the shifter is the last bit shifted
    // out
    // shifter_operand = Rm Logical_Shift_Left shift_imm
    // shifter_carry_out = Rm[32 - shift_imm]

  } else if (temp == SHIFTER_LSR_IMM_DECODE) {
    // printf("LSR imm\n");

  } else if (temp == SHIFTER_ASR_IMM_DECODE) {

  } else if (temp == SHIFTER_ROR_IMM_DECODE) {

    // printf("ROR imm\n");
  }

  temp = OP_CODE & SHIFTER_SHIFT_REG_MASK;

  if (temp == SHIFTER_LSL_REG_DECODE) {
    // printf("LSL reg\n");

  } else if (temp == SHIFTER_LSR_REG_DECODE) {
    // printf("lsr reg\n");

  } else if (temp == SHIFTER_ASR_REG_DECODE) {

  } else if (temp == SHIFTER_ROR_REG_DECODE) {
    // printf("ROR reg\n");
  }

  write_instruction_log(arm, "data process");
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
EOR_INST:
SUB_INST:
RSB_INST:
ADD_INST:
ADC_INST:
SBC_INST:
RSC_INST:
TST_INST:
TEQ_INST:
CMP_INST:
CMN_INST:
ORR_INST:
MOV_INST:
BIC_INST:
MVN_INST:

END:
  return 0;
}
