#include "arm.h"
#include "thumb_inst_decode.h"
#include <stdint.h>

#define OP_CODE arm->data_bus

int thumb_exec(Arm *arm) {

  uint32_t temp = 0;

DECODE:

  if ((OP_CODE & COND_BRANCH_MASK) == COND_BRANCH_DECODE) {

  } else if ((OP_CODE & UNCOND_BRANCH_MASK) == UNCOND_BRANCH_DECODE) {

  } else if ((OP_CODE & BRANCH_EXCHANGE_MASK) == BRANCH_EXCHANGE_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F1_MASK) == DATA_PROCESS_F1_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F2_MASK) == DATA_PROCESS_F2_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F3_MASK) == DATA_PROCESS_F3_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F4_MASK) == DATA_PROCESS_F4_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F5_MASK) == DATA_PROCESS_F5_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F6_MASK) == DATA_PROCESS_F6_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F7_MASK) == DATA_PROCESS_F7_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F8_MASK) == DATA_PROCESS_F8_DECODE) {
  }

  temp = OP_CODE & LOAD_STORE_F1_MASK;

  if (temp == LDR1_DECODE) {

  } else if (temp == LDRB1_DECODE) {

  } else if (temp == LDRH1_DECODE) {

  } else if (temp == STR1_DECODE) {

  } else if (temp == STRB1_DECODE) {

  } else if (temp == STRH1_DECODE) {

  } else if (temp == LDR3_DECODE) {
  } else if (temp == LDMIA_DECODE) {

  } else if (temp == STMIA_DECODE) {

  }

  temp = OP_CODE & LOAD_STORE_F2_MASK;

  if (temp == LDR2_DECODE) {

  } else if (temp == LDRB2_DECODE) {

  } else if (temp == LDRH2_DECODE) {

  } else if (temp == STR2_DECODE) {

  } else if (temp == STRB2_DECODE) {

  } else if (temp == STRH2_DECODE) {

  } else if (temp == LDRSB_DECODE) {

  } else if (temp == LDRSH_DECODE) {
  } else if (temp == PUSH) {

  } else if (temp == POP) {

  }

  if ((OP_CODE & SWI_MASK) == SWI_DECODE) {

  }

  return 0;
}
