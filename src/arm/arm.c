/* TODO - Check decoder code */
#include "arm.h"
#include "inst_decode.h"
#include <stdio.h>
#define DEBUG_ON

#define OP_CODE arm->data_bus
int arm_exec(Arm *arm) {

  static void *dp_inst_table[] = {&&AND_INST, &&EOR_INST, &&SUB_INST, &&RSB_INST,
                                  &&ADD_INST, &&ADC_INST, &&SBC_INST, &&RSC_INST,
                                  &&TST_INST, &&TEQ_INST, &&CMP_INST, &&CMN_INST,
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
  goto DECODE;

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
  } else if ((OP_CODE & DATA_PROCESS_MASK) == DATA_PROCESS_DECODE){
    // instructions can data processing, control or undefined or swp or swbp
    if ((OP_CODE & CONTROL_MASK) != CONTROL_DECODE) {
      // instructions are data processing
      goto DATA_PROCESS;
    }

    if ((OP_CODE & SWP_MASK) == SWP_DECODE) {
      // swp instruction
    } else if ((OP_CODE & SWPB_MASK) == SWPB_DECODE) {
      // swpb instruction
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
  }  else if ((OP_CODE & SWI_MASK) == SWI_DECODE) {
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

  return 0;
}
