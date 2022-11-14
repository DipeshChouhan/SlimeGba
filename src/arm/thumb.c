// TODO - Implement flag setting for data processing instructions !{IMPORTANT}
#include "arm.h"
#include "thumb_inst_decode.h"
#include <stdint.h>
#include <stdlib.h>

#define OP_CODE arm->data_bus

// vf1 = first source operand
// vf2 = second source operand

#define FLAGS_NZCV(_zf, _vf1, _vf2)                                            \
  temp = arm->cpsr & 0xFFFFFFF;                                                \
  temp |= (result & 0x80000000);                                               \
  temp |= ((_zf == 0) << 30);                                                  \
  temp |= ((result & 0x100000000) >> 3);                                       \
  temp |= (((_vf1 & 0x80000000) == (_vf2 & 0x80000000)) &&                     \
           ((result & 0x80000000) != (_vf1 & 0x80000000)))                     \
          << 28;

#define FLAGS_NZ(_nf, _zf)                                                     \
  temp = arm->cpsr & 0x3FFFFFFF;                                               \
  temp |= (_nf & 0x80000000);                                                  \
  temp |= ((_zf == 0) << 30);

#define FLAG_C(_to) arm->cpsr |= (arm->cpsr & (~0x20000000)) | (_to << 29);

int thumb_exec(Arm *arm) {

  uint32_t temp = 0;
  uint64_t result = 0;
  uint32_t rn = 0;
  uint32_t rd = 0;
  uint32_t rm = 0;
  uint32_t reg_count = 0;
  uint32_t *reg_p = NULL;
  uint32_t imm_value = 0;

  static void *dp_inst_table[] = {
      &&ADD3, &&SUB3, &&ADD1, &&SUB1, &&MOV1, &&CMP1, &&ADD2, &&SUB2, &&LSL1,
      &&LSR1, &&ASR1, &&AND,  &&EOR,  &&LSL2, &&LSR2, &&ASR2, &&ADC,  &&SBC,
      &&ROR,  &&TST,  &&NEG,  &&CMP2, &&CMN,  &&ORR,  &&MUL,  &&BIC,  &&MVN,
      &&ADD5, &&ADD6, &&ADD7, &&SUB4, &&ADD4, &&CMP3, &&MOV3, &&CPY};

DECODE:

  if ((OP_CODE & SWI_MASK) == SWI_DECODE) {
    // swi
  }

  if ((OP_CODE & COND_BRANCH_MASK) == COND_BRANCH_DECODE) {

  } else if ((OP_CODE & UNCOND_BRANCH_MASK) == UNCOND_BRANCH_DECODE) {

  } else if ((OP_CODE & BRANCH_EXCHANGE_MASK) == BRANCH_EXCHANGE_DECODE) {

  } else if ((OP_CODE & DATA_PROCESS_F1_MASK) == DATA_PROCESS_F1_DECODE) {
    if (IS_BIT_SET(OP_CODE, 9)) {
      rn = arm->general_regs[(OP_CODE >> 3) & 7];
      rm = arm->general_regs[(OP_CODE >> 6) & 7];
      reg_p = &arm->general_regs[OP_CODE & 7];
      goto SUB3;
    }
    goto ADD3;

  } else if ((OP_CODE & DATA_PROCESS_F2_MASK) == DATA_PROCESS_F2_DECODE) {
    if (IS_BIT_SET(OP_CODE, 9)) {
      rn = arm->general_regs[(OP_CODE >> 3) & 7];
      reg_p = &arm->general_regs[OP_CODE & 7];
      imm_value = (OP_CODE >> 6) & 7;
      goto SUB1;
    }
    goto ADD1;

  } else if ((OP_CODE & DATA_PROCESS_F3_MASK) == DATA_PROCESS_F3_DECODE) {
    reg_p = &arm->general_regs[(OP_CODE >> 8) & 7];
    imm_value = OP_CODE & 0xFF;
    goto *dp_inst_table[4 + ((OP_CODE >> 11) & 3)];

  } else if ((OP_CODE & DATA_PROCESS_F4_MASK) == DATA_PROCESS_F4_DECODE) {
    rm = arm->general_regs[(OP_CODE >> 3) & 7];
    reg_p = &arm->general_regs[OP_CODE & 7];
    imm_value = (OP_CODE >> 6) & 31;
    goto *dp_inst_table[8 + ((OP_CODE >> 11) & 3)];

  } else if ((OP_CODE & DATA_PROCESS_F5_MASK) == DATA_PROCESS_F5_DECODE) {
    rm = arm->general_regs[(OP_CODE >> 3) & 7];
    reg_p = &arm->general_regs[OP_CODE & 7];
    goto *dp_inst_table[11 + ((OP_CODE >> 6) & 0xF)];

  } else if ((OP_CODE & DATA_PROCESS_F6_MASK) == DATA_PROCESS_F6_DECODE) {
    reg_p = &arm->general_regs[(OP_CODE >> 8) & 7];
    imm_value = OP_CODE & 0xFF;
    reg_count = arm->mode * 16;
    if (IS_BIT_SET(OP_CODE, 11)) {
      rn = *arm->reg_table[reg_count + 13]; // sp
      goto ADD6;
    }
    rn = arm->general_regs[15]; // pc
    goto ADD5;

  } else if ((OP_CODE & DATA_PROCESS_F7_MASK) == DATA_PROCESS_F7_DECODE) {
    reg_count = arm->mode * 16;
    reg_p = arm->reg_table[reg_count + 13];
    imm_value = OP_CODE & 0x7F;
    if (IS_BIT_SET(OP_CODE, 7)) {
      goto SUB4;
    }
    goto ADD7;

  } else if ((OP_CODE & DATA_PROCESS_F8_MASK) == DATA_PROCESS_F8_DECODE) {
    reg_count = arm->mode * 16;
    temp = IS_BIT_SET(OP_CODE, 7) * 8;
    reg_p = arm->reg_table[reg_count + (OP_CODE & 7) + temp];
    temp = IS_BIT_SET(OP_CODE, 6) * 8;
    rm = *arm->reg_table[reg_count + ((OP_CODE >> 3) & 7) + temp];

    goto *dp_inst_table[31 + ((OP_CODE >> 8) & 3)];
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

ADD3:
  result = rn + rm;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, rn, rm);
ADD1:

  result = rn + imm_value;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, rn, imm_value);

  goto END;

ADD2:
  result = *reg_p + imm_value;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, imm_value);
  *reg_p = result;
  goto END;
SUB3:
  rm = (~rm) + 1;
  result = rn + rm;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, rn, rm);
SUB1:
  imm_value = (~imm_value) + 1;
  result = rn + imm_value;
  FLAGS_NZCV((result & 0xFFFFFFFF), rn, imm_value);
  *reg_p = result;
SUB2:
  imm_value = (~imm_value) + 1;
  result = *reg_p + imm_value;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, imm_value);
  *reg_p = result;
LSL1:
  if (imm_value == 0) {
    *reg_p = rm;
  } else {
    arm->cpsr |=
        (arm->cpsr & (~0x20000000)) | (IS_BIT_SET(rm, (32 - imm_value)) << 29);
    *reg_p = rm << imm_value;
  }
  FLAGS_NZ(*reg_p, *reg_p);
CMP1:
  imm_value = (~imm_value) + 1;
  result = *reg_p + imm_value;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, imm_value);

MOV1:
  *reg_p = imm_value;
  FLAGS_NZ(*reg_p, *reg_p);

LSR1:
  if (imm_value == 0) {
    FLAG_C(IS_BIT_SET(rm, 31));
    *reg_p = 0;
  } else {
    FLAG_C(IS_BIT_SET(rm, (imm_value - 1)));
    *reg_p = rm >> imm_value;
  }
  FLAGS_NZ(*reg_p, *reg_p);
ASR1:
  if (imm_value == 0) {
    FLAG_C(IS_BIT_SET(rm, 31));
    if (!IS_BIT_SET(rm, 31)) {
      *reg_p = 0;
    } else {
      *reg_p = 0xFFFFFFFF;
    }
  } else {
    FLAG_C(IS_BIT_SET(rm, (imm_value - 1)));
    *reg_p = (rm >> imm_value) |
             (IS_BIT_SET(rm, 31) * (0xFFFFFFFF << (32 - imm_value)));
  }
  FLAGS_NZ(*reg_p, *reg_p);

AND:
  *reg_p = (*reg_p) & rm;
  FLAGS_NZ(*reg_p, *reg_p);
  goto END;
EOR:
  *reg_p = (*reg_p) ^ rm;
  FLAGS_NZ(*reg_p, *reg_p);
  goto END;
LSL2:
  imm_value = rm & 0xFF;
  if (imm_value == 0) {
    // unaffected
  } else if (imm_value < 32) {

    arm->cpsr |= (arm->cpsr & (~0x20000000)) |
                 (IS_BIT_SET(*reg_p, (32 - imm_value)) << 29);
    *reg_p = *reg_p << imm_value;

  } else if (imm_value == 32) {

    arm->cpsr |= (arm->cpsr & (~0x20000000)) | (IS_BIT_SET(*reg_p, 0) << 29);
    *reg_p = 0;

  } else {
    arm->cpsr &= (~0x20000000);
    *reg_p = 0;
  }
  FLAGS_NZ(*reg_p, *reg_p);
LSR2:

  imm_value = rm & 0xFF;
  if (imm_value == 0) {
  } else if (imm_value < 32) {
    FLAG_C(IS_BIT_SET(*reg_p, (imm_value - 1)));
    *reg_p = *reg_p >> imm_value;

  } else if (imm_value == 32) {
    FLAG_C(IS_BIT_SET(*reg_p, 31));
    *reg_p = 0;

  } else {
    FLAG_C(0);
    *reg_p = 0;
  }
  FLAGS_NZ(*reg_p, *reg_p);
ASR2:
  rm = rm & 0xFF;
  if (rm == 0) {

  } else if (rm < 32) {
    FLAG_C(IS_BIT_SET(*reg_p, (rm - 1)));
    *reg_p =
        (*reg_p >> rm) | (IS_BIT_SET(*reg_p, 31) * (0xFFFFFFFF << (32 - rm)));
  } else {
    FLAG_C(IS_BIT_SET(*reg_p, 31));
    if (IS_BIT_SET(*reg_p, 31)) {
      *reg_p = 0;
    } else {
      *reg_p = 0xFFFFFFFF;
    }
  }
  FLAGS_NZ(*reg_p, *reg_p);
ADC:
  result = *reg_p + rm + IS_BIT_SET(arm->cpsr, CF_BIT);
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
  *reg_p = result;
SBC:
  rm = (~rm) + IS_BIT_SET(arm->cpsr, CF_BIT);
  result = *reg_p + rm;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
  *reg_p = result;
ROR:
  if ((rm & 0xFF) == 0) {

  } else if ((rm & 0xF) == 0) {

  } else {
    rm = rm & 0xF;
    *reg_p = ROTATE_RIGHT32(*reg_p, rm);
  }
TST:
  result = *reg_p & rm;
  FLAGS_NZ(result, (result & 0xFFFFFFFF));
NEG:
  rm = (~rm) + 1;
  result = rm;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, 0, rm);
CMP2:
  rm = (~rm) + 1;
  result = *reg_p + rm;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);

CMN:
  result = *reg_p + rm;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
ORR:
  *reg_p = (*reg_p) | rm;
  FLAGS_NZ(*reg_p, *reg_p);
MUL:
  *reg_p = (rm * (*reg_p));
  FLAGS_NZ(*reg_p, *reg_p);
BIC:
  *reg_p = *reg_p & (~rm);
  FLAGS_NZ(*reg_p, *reg_p);
MVN:
  *reg_p = (~rm);
  FLAGS_NZ(*reg_p, *reg_p);
ADD5:
  *reg_p = (rn & 0xFFFFFFFC) + (imm_value * 4);
  goto END;
ADD6:
  *reg_p = rn + (imm_value << 2);
  goto END;
ADD7:
  *reg_p = *reg_p + (imm_value << 2);
  goto END;
SUB4:
  imm_value <<= 2;
  *reg_p = *reg_p + (~imm_value) + 1;
ADD4:
  *reg_p = *reg_p + rm;
  goto END;
CMP3:
  rm = (~rm) + 1;
  result = *reg_p + rm;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
MOV3:
  *reg_p = rm;
CPY:

END:
  return 0;
}
