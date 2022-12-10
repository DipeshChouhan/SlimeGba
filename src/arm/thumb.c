// TODO - Implement flag setting for data processing instructions !{DONE NOT
// TESTED}
// TODO - check sign extending in Branch instruction !{DONE}
// TODO - check instruction fetch
// TESTED}
// TODO check ASR instructions
// TODO check branching is Rd is PC
#include "../gba/gba.h"
#include "../memory/memory.h"
#include "arm.h"
#include "disassembler.h"
#include "thumb_inst_decode.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define DEBUG_ON

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
          << 28;                                                               \
  arm->cpsr = temp;

#define FLAGS_NZ(_nf, _zf)                                                     \
  temp = arm->cpsr & 0x3FFFFFFF;                                               \
  temp |= (_nf & 0x80000000);                                                  \
  temp |= ((_zf == 0) << 30);                                                  \
  arm->cpsr = temp;

#define FLAG_C(_to) arm->cpsr = SET_BIT(arm->cpsr, 29, _to);

#define THUMB_FETCH(_address, _dest)                                           \
  MEM_READ(_address, _dest, mem_read16);                                       \
  arm->general_regs[15] = _address + 4;                                        \
  _address += 2;

int thumb_exec(Arm *arm) {
  // if (arm->mode == UND) {
  //   printf("exit_code - %d\n", arm->general_regs[1]);
  //   exit(0);
  // }

  uint32_t temp = 0;
  uint64_t result = 0;
  uint32_t rn = 0;
  uint32_t rd = 0;
  uint32_t rm = 0;
  uint32_t reg_count = 0;
  uint32_t *reg_p = NULL;
  uint32_t imm_value = 0;
  uint32_t address = 0;

  static void *dp_inst_table[] = {
      &&ADD3, &&SUB3, &&ADD1, &&SUB1, &&MOV1, &&CMP1, &&ADD2, &&SUB2, &&LSL1,
      &&LSR1, &&ASR1, &&AND,  &&EOR,  &&LSL2, &&LSR2, &&ASR2, &&ADC,  &&SBC,
      &&ROR,  &&TST,  &&NEG,  &&CMP2, &&CMN,  &&ORR,  &&MUL,  &&BIC,  &&MVN,
      &&ADD5, &&ADD6, &&ADD7, &&SUB4, &&ADD4, &&CMP3, &&MOV3, &&CPY};

  static void *cond_field_table[] = {
      &&CHECK_EQ, &&CHECK_NE, &&CHECK_CS_HS, &&CHECK_CC_LO, &&CHECK_MI,
      &&CHECK_PL, &&CHECK_VS, &&CHECK_VC,    &&CHECK_HI,    &&CHECK_LS,
      &&CHECK_GE, &&CHECK_LT, &&CHECK_GT,    &&CHECK_LE,    &&UNDEFINED};

  INTERRUPT_REQUEST();

  THUMB_FETCH(arm->curr_instruction, arm->data_bus);
  printf("OP_CODE - %X\n", OP_CODE);

DECODE:

  if ((OP_CODE & SWI_MASK) == SWI_DECODE) {
    // printf("SWI\n");
    // write_decoder_log(arm, "SWI");
    // swi
    SWI_INSTRUCTION(arm);
    goto END;
  }

  if ((OP_CODE & COND_BRANCH_MASK) == COND_BRANCH_DECODE) {
    // write_decoder_log(arm, "COND_BRANCH");
    // B1
    //
  #ifdef DEBUG_ON
    write_instruction_log(arm, "B");
    printf("b opcode - %X\n", OP_CODE);
#endif
    goto *cond_field_table[(OP_CODE >> 8) & 0xF];

  CHECK_EQ:
    if (IS_BIT_SET(arm->cpsr, ZF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_NE:
    if (IS_BIT_SET(arm->cpsr, ZF_BIT)) {
      goto END;
    }
    goto B1_INST;
  CHECK_CS_HS:
    if (IS_BIT_SET(arm->cpsr, CF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_CC_LO:
    if (IS_BIT_SET(arm->cpsr, CF_BIT)) {
      goto END;
    }
    goto B1_INST;
  CHECK_MI:
    if (IS_BIT_SET(arm->cpsr, NF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_PL:
    if (IS_BIT_SET(arm->cpsr, NF_BIT)) {
      goto END;
    }
    goto B1_INST;
  CHECK_VS:
    if (IS_BIT_SET(arm->cpsr, VF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_VC:
    if (IS_BIT_SET(arm->cpsr, VF_BIT)) {
      goto END;
    }
    goto B1_INST;
  CHECK_HI:
    // c set and z clear
    if ((arm->cpsr & 0x60000000) == 0x20000000) {
      goto B1_INST;
    }
    goto END;
  CHECK_LS:
    if ((IS_BIT_SET(arm->cpsr, CF_BIT) == 0) || IS_BIT_SET(OP_CODE, ZF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_GE:
    if (GET_BIT(arm->cpsr, NF_BIT) == GET_BIT(OP_CODE, VF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_LT:
    if (GET_BIT(arm->cpsr, NF_BIT) != GET_BIT(OP_CODE, VF_BIT)) {
      goto B1_INST;
    }
    goto END;
  CHECK_GT:
    if ((IS_BIT_SET(arm->cpsr, ZF_BIT) == 0) &&
        (GET_BIT(arm->cpsr, NF_BIT) == GET_BIT(arm->cpsr, VF_BIT))) {
      goto B1_INST;
    }
    goto END;
  CHECK_LE:
    if (IS_BIT_SET(arm->cpsr, ZF_BIT) ||
        (GET_BIT(arm->cpsr, NF_BIT) != GET_BIT(arm->cpsr, VF_BIT))) {
      goto B1_INST;
    }
    goto END;

  B1_INST:
    // write_decoder_log(arm, "B1");
    imm_value = OP_CODE & 0xFF;
    imm_value = SIGN_EXTEND(imm_value, 7);
    arm->general_regs[15] += (imm_value << 1);
    arm->curr_instruction = arm->general_regs[15];
    goto END;

  } else if ((OP_CODE & UNCOND_BRANCH_MASK) == UNCOND_BRANCH_DECODE) {
    // write_decoder_log(arm, "UNCOND_BRANCH");
    temp = OP_CODE & 0x1800;       // bits 11 and 12
                                   // 0xFFFFF800
    imm_value = (OP_CODE & 0x7FF); // signed_immed_11
    if (temp == 0x0) {
      // B2
      imm_value = SIGN_EXTEND(imm_value, 10);
      arm->general_regs[15] += (imm_value << 1);
      arm->curr_instruction = arm->general_regs[15];
      goto END;
    } else if (temp == 0x1000) {
      // BL H = 10 form
      imm_value = SIGN_EXTEND(imm_value, 10);
      reg_count = arm->mode * 16;
      *arm->reg_table[reg_count + 14] =
          arm->general_regs[15] + (imm_value << 12);

#ifdef DEBUG_ON
      write_instruction_log(arm, "BL10");
#endif
      goto END;
    } else if (temp == 0x1800) {
      // BL H = 11 form
      reg_count = arm->mode * 16;
      arm->general_regs[15] =
          *arm->reg_table[reg_count + 14] + (imm_value << 1);
      // TODO check correctness
      // LR = (address of next instruction) | 1
      *arm->reg_table[reg_count + 14] = (arm->curr_instruction | 1);
      arm->curr_instruction = arm->general_regs[15];

#ifdef DEBUG_ON
      write_instruction_log(arm, "BL11");
#endif
      goto END;
    }

    goto UNDEFINED;

  } else if ((OP_CODE & BRANCH_EXCHANGE_MASK) == BRANCH_EXCHANGE_DECODE) {
    // write_decoder_log(arm, "BX");
    reg_count = arm->mode * 16;

    rm = *arm->reg_table[reg_count + ((OP_CODE >> 3) & 0xF)];
    arm->cpsr = (arm->cpsr & 0xFFFFFFDF) | ((rm & 1) << 5);
    arm->state = rm & 1;
    arm->general_regs[15] = rm & 0xFFFFFFFE;
    arm->curr_instruction = arm->general_regs[15];
    printf("rm - %d\n", arm->curr_instruction);
    write_instruction_log(arm, "BX");
    goto END;

  } else if ((OP_CODE & DATA_PROCESS_F1_MASK) == DATA_PROCESS_F1_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    rn = arm->general_regs[(OP_CODE >> 3) & 7];
    rm = arm->general_regs[(OP_CODE >> 6) & 7];
    reg_p = &arm->general_regs[OP_CODE & 7];
    if (IS_BIT_SET(OP_CODE, 9)) {
      goto SUB3;
    }
    goto ADD3;

  } else if ((OP_CODE & DATA_PROCESS_F2_MASK) == DATA_PROCESS_F2_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    rn = arm->general_regs[(OP_CODE >> 3) & 7];
    reg_p = &arm->general_regs[OP_CODE & 7];
    imm_value = (OP_CODE >> 6) & 7;
    if (IS_BIT_SET(OP_CODE, 9)) {
      goto SUB1;
    }

    goto ADD1;

  } else if ((OP_CODE & DATA_PROCESS_F3_MASK) == DATA_PROCESS_F3_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    reg_p = &arm->general_regs[(OP_CODE >> 8) & 7];
    imm_value = OP_CODE & 0xFF;
    goto *dp_inst_table[4 + ((OP_CODE >> 11) & 3)];

  } else if ((OP_CODE & DATA_PROCESS_F4_MASK) == DATA_PROCESS_F4_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    rm = arm->general_regs[(OP_CODE >> 3) & 7];
    reg_p = &arm->general_regs[OP_CODE & 7];
    imm_value = (OP_CODE >> 6) & 31;
    goto *dp_inst_table[8 + ((OP_CODE >> 11) & 3)];

  } else if ((OP_CODE & DATA_PROCESS_F5_MASK) == DATA_PROCESS_F5_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    rm = arm->general_regs[(OP_CODE >> 3) & 7];
    reg_p = &arm->general_regs[OP_CODE & 7];
    goto *dp_inst_table[11 + ((OP_CODE >> 6) & 0xF)];

  } else if ((OP_CODE & DATA_PROCESS_F6_MASK) == DATA_PROCESS_F6_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
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
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    reg_count = arm->mode * 16;
    reg_p = arm->reg_table[reg_count + 13];
    imm_value = OP_CODE & 0x7F;
    if (IS_BIT_SET(OP_CODE, 7)) {
      goto SUB4;
    }
    goto ADD7;

  } else if ((OP_CODE & DATA_PROCESS_F8_MASK) == DATA_PROCESS_F8_DECODE) {
    // write_decoder_log(arm, "Thumb Data Processing Instruction");
    reg_count = arm->mode * 16;
    temp = IS_BIT_SET(OP_CODE, 7) * 8;
    reg_p = arm->reg_table[reg_count + (OP_CODE & 7) + temp];
    temp = IS_BIT_SET(OP_CODE, 6) * 8;
    rm = *arm->reg_table[reg_count + ((OP_CODE >> 3) & 7) + temp];

    goto *dp_inst_table[31 + ((OP_CODE >> 8) & 3)];
  }

#define immed_5 ((OP_CODE >> 6) & 0x1F)
#define RN_F1 arm->general_regs[((OP_CODE >> 3) & 7)]
#define RD_F1 arm->general_regs[(OP_CODE & 7)]
#define immed_8 (OP_CODE & 0xFF)
#define RD_F3 arm->general_regs[(OP_CODE >> 8) & 7]
#define RN_F3 arm->general_regs[(OP_CODE >> 8) & 7]
#define RM_F2 arm->general_regs[(OP_CODE >> 6) & 7]

  temp = OP_CODE & LOAD_STORE_F1_MASK;

  if (temp == LDR1_DECODE) {
    // write_decoder_log(arm, "LDR1");
    imm_value = immed_5 * 4;
    address = RN_F1 + imm_value;
    MEM_READ(address, RD_F1, mem_read32);
    goto END;

  } else if (temp == LDRB1_DECODE) {
    // write_decoder_log(arm, "LDRB1");
    address = RN_F1 + immed_5;
    MEM_READ(address, RD_F1, mem_read8);
    goto END;

  } else if (temp == LDRH1_DECODE) {
    // write_decoder_log(arm, "LDRH1");
    imm_value = immed_5 * 2;
    address = RN_F1 + imm_value;
    MEM_READ(address, RD_F1, mem_read16);
    goto END;

  } else if (temp == STR1_DECODE) {
    // write_decoder_log(arm, "STR1");
    imm_value = immed_5 * 4;
    address = RN_F1 + imm_value;
    MEM_WRITE(address, RD_F1, mem_write32);
    goto END;

  } else if (temp == STRB1_DECODE) {
    // write_decoder_log(arm, "STRB1");
    address = RN_F1 + immed_5;
    MEM_WRITE(address, RD_F1, mem_write8);
    goto END;

  } else if (temp == STRH1_DECODE) {
    // write_decoder_log(arm, "STRH1");
    imm_value = immed_5 * 2;
    address = RN_F1 + imm_value;
    MEM_WRITE(address, RD_F1, mem_write16);
    goto END;

  } else if (temp == LDR3_DECODE) {
    // write_decoder_log(arm, "LDR3");

    imm_value = immed_8 * 4;
    address = (arm->general_regs[15] & 0xFFFFFFFC) + imm_value;
    MEM_READ(address, RD_F3, mem_read32);
    goto END;
  } else if (temp == LDR4_DECODE) {
    // write_decoder_log(arm, "LDR4");
    reg_count = arm->mode * 16;
    imm_value = immed_8 * 4;
    address = *arm->reg_table[reg_count + 13] + imm_value;
    MEM_READ(address, RD_F3, mem_read32);
    goto END;

  } else if (temp == STR3_DECODE) {
    // write_decoder_log(arm, "STR3");
    reg_count = arm->mode * 16;
    imm_value = immed_8 * 4;
    address = *arm->reg_table[reg_count + 13] + imm_value;
    MEM_WRITE(address, RD_F3, mem_write32);
    goto END;

  } else if (temp == LDMIA_DECODE) {

    imm_value = immed_8; // reglist
    address = RN_F3;     // start address
    reg_count = 0;       // register count
    rm = 0;              // number of set bits
    while (imm_value) {
      if (imm_value & 1) {
        MEM_READ(address, arm->general_regs[reg_count], mem_read32);
        address += 4;
        ++rm;
      }
      imm_value >>= 1;
      ++reg_count;
    }

    assert(((RN_F3 + (rm * 4)) - 4) == address - 4);
    RN_F3 = RN_F3 + (rm * 4);
    goto END;

  } else if (temp == STMIA_DECODE) {

    imm_value = immed_8; // reglist
    address = RN_F3;     // start address
    reg_count = 0;       // register count
    rm = 0;              // number of set bits
    while (imm_value) {
      if (imm_value & 1) {
        MEM_WRITE(address, arm->general_regs[reg_count], mem_write32);
        address += 4;
        ++rm;
      }
      imm_value >>= 1;
      ++reg_count;
    }

    assert(((RN_F3 + (rm * 4)) - 4) == address - 4);
    RN_F3 = RN_F3 + (rm * 4);
    goto END;
  }

  address = RN_F1 + RM_F2;
  temp = OP_CODE & LOAD_STORE_F2_MASK;

  if (temp == LDR2_DECODE) {
    // write_decoder_log(arm, "LDR2");
    MEM_READ(address, RD_F1, mem_read32);
    goto END;

  } else if (temp == LDRB2_DECODE) {
    // write_decoder_log(arm, "LDRB2");
    MEM_READ(address, RD_F1, mem_read8);
    goto END;
  } else if (temp == LDRH2_DECODE) {
    // write_decoder_log(arm, "LDRH2");
    MEM_READ(address, RD_F1, mem_read16);
    goto END;

  } else if (temp == STR2_DECODE) {
    // write_decoder_log(arm, "STR2");
    MEM_WRITE(address, RD_F1, mem_write32);
    goto END;
  } else if (temp == STRB2_DECODE) {
    // write_decoder_log(arm, "STRB2");
    MEM_WRITE(address, RD_F1, mem_write8);
    goto END;

  } else if (temp == STRH2_DECODE) {
    // write_decoder_log(arm, "STRH2");
    MEM_WRITE(address, RD_F1, mem_write16);
    goto END;

  } else if (temp == LDRSB_DECODE) {
    // write_decoder_log(arm, "LDRSB");
    MEM_READ(address, imm_value, mem_read8);
    imm_value = SIGN_EXTEND(imm_value, 7);
    RD_F1 = imm_value;
    goto END;

  } else if (temp == LDRSH_DECODE) {
    // write_decoder_log(arm, "LDRSH");
    MEM_READ(address, imm_value, mem_read16);
    imm_value = SIGN_EXTEND(imm_value, 15);
    RD_F1 = imm_value;
    goto END;
  } else if (temp == PUSH) {
    // write_decoder_log(arm, "PUSH");
    imm_value = immed_8; // reg list

    rm = 0; // number of set bits
    while (imm_value) {
      rm += imm_value & 1;
      imm_value >>= 1;
    }
    imm_value = immed_8;
    rn = IS_BIT_SET(OP_CODE, 8); // R bit
    address = *arm->reg_table[(arm->mode * 16) + 13] - 4 * (rn + rm);

    reg_count = 0;
    while (imm_value) {
      if (imm_value & 1) {
        MEM_WRITE(address, arm->general_regs[reg_count], mem_write32);
        address += 4;
      }
      ++reg_count;
      imm_value >>= 1;
    }

    reg_count = arm->mode * 16;
    if (rn) {
      MEM_WRITE(address, *arm->reg_table[reg_count + 14], mem_write32);
      address += 4;
    }
    assert((*arm->reg_table[reg_count + 13] - 4) == address - 4);
    *arm->reg_table[reg_count + 13] -= 4 * (rn + rm);
    goto END;

  } else if (temp == POP) {
    // write_decoder_log(arm, "POP");
    address = *arm->reg_table[(arm->mode * 16) + 13]; // SP
    imm_value = immed_8;                              // reg list
    rn = IS_BIT_SET(OP_CODE, 8);                      // R Bit
    reg_count = 0;
    rm = 0; // number of set bits
    while (imm_value) {
      if (imm_value & 1) {
        MEM_READ(address, arm->general_regs[reg_count], mem_read32);
        address += 4;
        ++rm;
      }
      imm_value >>= 1;
      ++reg_count;
    }

    if (rn) {
      MEM_READ(address, imm_value, mem_read32);
      arm->general_regs[15] = imm_value & 0xFFFFFFFE;
      arm->curr_instruction = arm->general_regs[15];
      address += 4;
    }
    reg_count = arm->mode * 16;
    assert((*arm->reg_table[reg_count + 13] + 4 * (rn + rm)) == address);
    *arm->reg_table[reg_count + 13] =
        *arm->reg_table[reg_count + 13] + 4 * (rn + rm);
    goto END;
  }

  goto UNDEFINED;

ADD3:
  result = (uint64_t)rn + rm;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, rn, rm);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD3");
#endif
  goto END;
ADD1:
  result = (uint64_t)rn + imm_value;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, rn, imm_value);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD1");
#endif

  goto END;

ADD2:
  result = *reg_p + (uint64_t)imm_value;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, imm_value);
  *reg_p = result;
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD2");
#endif
  goto END;
SUB3:
  rm = (~rm);
  result = rn + ((uint64_t)rm + 1);
  *reg_p = result;
  rm += 1;
  FLAGS_NZCV(*reg_p, rn, rm);

#ifdef DEBUG_ON
  write_instruction_log(arm, "SUB3");
#endif
  goto END;
SUB1:
  imm_value = (~imm_value);
  result = rn + ((uint64_t)imm_value + 1);
  imm_value += 1;
  FLAGS_NZCV((result & 0xFFFFFFFF), rn, imm_value);
  *reg_p = result;
#ifdef DEBUG_ON
  write_instruction_log(arm, "SUB1");
#endif
  goto END;
SUB2:
  imm_value = (~imm_value);
  result = *reg_p + ((uint64_t)imm_value + 1);
  imm_value += 1;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, imm_value);
  *reg_p = result;
#ifdef DEBUG_ON
  write_instruction_log(arm, "SUB2");
#endif
  goto END;
LSL1:
  if (imm_value == 0) {
    *reg_p = rm;
  } else {
    FLAG_C(IS_BIT_SET(rm, (32 - imm_value)));
    *reg_p = rm << imm_value;
  }
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "LSL1");
#endif
  goto END;
CMP1:
  imm_value = (~imm_value);
  result = *reg_p + ((uint64_t)imm_value + 1);
  imm_value += 1;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, imm_value);
#ifdef DEBUG_ON
  write_instruction_log(arm, "CMP1");
#endif
  goto END;

MOV1:
  *reg_p = imm_value;
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "MOV1");
#endif
  goto END;

LSR1:
  if (imm_value == 0) {
    FLAG_C(IS_BIT_SET(rm, 31));
    *reg_p = 0;
  } else {
    FLAG_C(IS_BIT_SET(rm, (imm_value - 1)));
    *reg_p = rm >> imm_value;
  }
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "LSR1");
#endif
  goto END;
ASR1:
  if (imm_value == 0) {
    FLAG_C(IS_BIT_SET(rm, 31));
    if (IS_BIT_SET(rm, 31)) {
      *reg_p = 0xFFFFFFFF;
    } else {
      *reg_p = 0;
    }
  } else {
    FLAG_C(IS_BIT_SET(rm, (imm_value - 1)));
    *reg_p = ASR_32(rm, imm_value);
    // *reg_p = (rm >> imm_value) |
    //          (IS_BIT_SET(rm, 31) * (0xFFFFFFFF << (32 - imm_value)));
  }
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ASR1");
#endif
  goto END;

AND:
  *reg_p = (*reg_p) & rm;
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "AND");
#endif
  goto END;
EOR:
  *reg_p = (*reg_p) ^ rm;
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "EOR");
#endif
  goto END;
LSL2:
  imm_value = rm & 0xFF;
  if (imm_value == 0) {
    // unaffected
  } else if (imm_value < 32) {

    FLAG_C(IS_BIT_SET(*reg_p, (32 - imm_value)));
    *reg_p = *reg_p << imm_value;

  } else if (imm_value == 32) {

    FLAG_C((*reg_p & 1));
    *reg_p = 0;

  } else {
    arm->cpsr &= (~0x20000000);
    *reg_p = 0;
  }
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "LSL2");
#endif
  goto END;
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
#ifdef DEBUG_ON
  write_instruction_log(arm, "LSR2");
#endif
  goto END;
ASR2:
  rm = rm & 0xFF;
  if (rm == 0) {

  } else if (rm < 32) {
    FLAG_C(IS_BIT_SET(*reg_p, (rm - 1)));
    *reg_p = ASR_32(*reg_p, rm);
    // *reg_p =
    //     (*reg_p >> rm) | (IS_BIT_SET(*reg_p, 31) * (0xFFFFFFFF << (32 -
    //     rm)));
  } else {
    FLAG_C(IS_BIT_SET(*reg_p, 31));
    if (IS_BIT_SET(*reg_p, 31)) {
      *reg_p = 0;
    } else {
      *reg_p = 0xFFFFFFFF;
    }
  }
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ASR2");
#endif
  goto END;
ADC:
  result = *reg_p + (uint64_t)rm + IS_BIT_SET(arm->cpsr, CF_BIT);
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
  *reg_p = result;
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADC");
#endif
  goto END;
SBC:
  rm = (~rm);
  result = *reg_p + ((uint64_t)rm + 1) - IS_BIT_NOT_SET(arm->cpsr, CF_BIT);
  rm += 1;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
  *reg_p = result;
#ifdef DEBUG_ON
  write_instruction_log(arm, "SBC");
#endif
  goto END;
ROR:
  if ((rm & 0xFF) == 0) {

  } else if ((rm & 0xF) == 0) {
    FLAG_C(IS_BIT_SET(*reg_p, 31));
  } else {
    rm = rm & 0xF;
    FLAG_C(IS_BIT_SET(*reg_p, (rm - 1)));
    *reg_p = ROTATE_RIGHT32(*reg_p, rm);
  }
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ROR");
#endif
  goto END;
TST:
  result = *reg_p & rm;
  FLAGS_NZ(result, (result & 0xFFFFFFFF));
#ifdef DEBUG_ON
  write_instruction_log(arm, "TST");
#endif
  goto END;
NEG:
  rm = (~rm) + 1;
  result = rm;
  *reg_p = result;
  FLAGS_NZCV(*reg_p, 0, rm);
#ifdef DEBUG_ON
  write_instruction_log(arm, "NEG");
#endif
  goto END;
CMP2:
  rm = (~rm);
  result = *reg_p + ((uint64_t)rm + 1);
  rm += 1;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);

#ifdef DEBUG_ON
  write_instruction_log(arm, "CMP2");
#endif
  goto END;

CMN:
  result = *reg_p + rm;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
#ifdef DEBUG_ON
  write_instruction_log(arm, "CMN");
#endif
  goto END;
ORR:
  *reg_p = (*reg_p) | rm;
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ORR");
#endif
  goto END;
MUL:
  *reg_p = (rm * (*reg_p));
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "MUL");
#endif
  goto END;
BIC:
  *reg_p = *reg_p & (~rm);
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "BIC");
#endif
  goto END;
MVN:
  *reg_p = (~rm);
  FLAGS_NZ(*reg_p, *reg_p);
#ifdef DEBUG_ON
  write_instruction_log(arm, "MVN");
#endif
  goto END;
ADD5:
  *reg_p = (rn & 0xFFFFFFFC) + (imm_value * 4);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD5");
#endif
  goto END;
ADD6:
  *reg_p = (uint64_t)rn + (imm_value << 2);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD6");
#endif
  goto END;
ADD7:
  *reg_p = (uint64_t)*reg_p + (imm_value << 2);
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD7");
#endif
  goto END;
SUB4:
  imm_value <<= 2;
  imm_value = ~imm_value;
  // printf("imm_value - %d\n", imm_value);
  *reg_p = *reg_p + ((uint64_t)imm_value + 1);
  imm_value += 1;
#ifdef DEBUG_ON
  write_instruction_log(arm, "SUB4");
#endif
  goto END;
ADD4:
  *reg_p = *reg_p + (uint64_t)rm;
  if (reg_p == arm->reg_table[15]) {
    arm->general_regs[15] = *reg_p & 0xFFFFFFFE;
    arm->curr_instruction = arm->general_regs[15];
  }
#ifdef DEBUG_ON
  write_instruction_log(arm, "ADD4");
#endif
  goto END;
CMP3:
  rm = (~rm);
  result = *reg_p + ((uint64_t)rm + 1);
  rm += 1;
  FLAGS_NZCV((result & 0xFFFFFFFF), *reg_p, rm);
#ifdef DEBUG_ON
  write_instruction_log(arm, "CMP3");
#endif
  goto END;
MOV3:
  *reg_p = rm;
  if (reg_p == arm->reg_table[15]) {
    *reg_p &= 0xFFFFFFFE;
    arm->curr_instruction = *reg_p;
  }

#ifdef DEBUG_ON
  write_instruction_log(arm, "MOV3");
#endif
  goto END;
CPY:
UNDEFINED:
  printf("Undefined\n");
  exit(1);
END:
  return 0;
}
