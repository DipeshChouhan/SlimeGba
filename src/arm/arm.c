/* TODO - Check decoder code */
#include "arm.h"
#include "inst_decode.h"
#include <stdio.h>

#define OP_CODE arm->data_bus
int arm_exec(Arm *arm) {

  // static void *dp_inst_table[] = {&&DO_AND, &&DO_EOR, &&DO_SUB, &&DO_RSB,
  //                                 &&DO_ADD, &&DO_ADC, &&DO_SBC, &&DO_RSC,
  //                                 &&DO_TST, &&DO_TEQ, &&DO_CMP, &&DO_CMN,
  //                                 &&DO_ORR, &&DO_MOV, &&DO_BIC, &&DO_MVN};
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
  } else if ((OP_CODE & LOAD_STORE_H_D_S_MASK) == LOAD_STORE_H_D_S_DECODE) {
    // Load and store halfword or doubleword, and load signed byte instructions
  } else if ((OP_CODE & DATA_PROCESS_MASK) != DATA_PROCESS_DECODE) {
    // Data processing instructions
  } else if ((OP_CODE & LOAD_STORE_W_U_MASK) == LOAD_STORE_W_U_DECODE) {
    // Load and store word or unsigned byte instructions and media instructions and architecturally undefined instructions
  } else if ((OP_CODE & LOAD_STORE_M_MASK) == LOAD_STORE_M_DECODE) {
    // Load and Store Multiple instructions
  } else if ((OP_CODE & BRANCH_LINK_MASK) == BRANCH_LINK_DECODE) {
    // Branch and branch link instructions
  } else if ((OP_CODE & CONTROL_MASK) == CONTROL_DECODE) {
    // control instructions and udefined instructions
  } else if ((OP_CODE & SWI_MASK) == SWI_DECODE) {
    // swi instruction
  } else if ((OP_CODE & COPROCESSOR_MASK) == COPROCESSOR_DECODE) {
    // coprocessor instructions
  } else {
    // undefined, unimplemented
  }

MULTIPLY:
LOAD_STORE_H_D_S:
LOAD_STORE_W_U:
LOAD_STORE_M:
DATA_PROCESS:
SWI:
CONTROL:
BRANCH_LINK:
COPROCESSOR:
UNDEFINED:
UNCONDITIONAL:
  // unpredictable
  return 0;
}
