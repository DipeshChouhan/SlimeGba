/* TODO - Check decoder code */
#include <stdio.h>
#include "arm.h"

#define NF_SET(_reg) ((_reg & (1 << 31)) > 0)

#define ZF_SET(_reg) ((_reg & (1 << 30)) > 0)

#define CF_SET(_reg) ((_reg & (1 << 29)) > 0)

#define VF_SET(_reg) ((_reg & (1 << 28)) > 0)
#define UNCONDITIONAL_ENCODE 0b11110000000000000000000000000000

#define MULTIPLY_ENCODE 0b00001111000000000000000011110000
#define MULTIPLY_DECODE 0b00000000000000000000000010010000

#define LOAD_STORE_ENCODE 0b00001110000000000000000010010000
#define LOAD_STORE_DECODE 0b00000000000000000000000010010000
#define LOAD_STORE_ENCODE1 0b00000001000000000000000001100000

#define LOAD_STORE_MULTIPLE_ENCODE 0b00001110000000000000000000000000
#define LOAD_STORE_MULTIPLE_DECODE 0b00001000000000000000000000000000

#define LOAD_STORE_IM_ENCODE 0b00001110000000000000000000000000
#define LOAD_STORE_IM_DECODE 0b00000100000000000000000000000000

#define LOAD_STORE_REG_ENCODE 0b00001110000000000000000000010000
#define LOAD_STORE_REG_DECODE 0b00000110000000000000000000000000

#define CONTROL_DATA_ENCODE 0b00001100000000000000000000000000

#define CONTROL_DSP_ENCODE 0b00000001100100000000000000000000
#define CONTROL_DSP_DECODE 0b00000001000000000000000000000000

#define CONTROL_DSP_ENCODE1 0b00000010000000000000000010010000
#define CONTORL_DSP_DECODE1 0b00000000000000000000000010010000

#define BX_ENCODE 0b00001111111100000000000011110000
#define BX_DECODE 0b00000001001000000000000000010000
#define BBL_ENCODE 0b00001110000000000000000000000000
#define BBL_DECODE 0b00001010000000000000000000000000

#define COPROC_DATA_ENCODE 0b00001111000000000000000000010000
#define COPROC_DATA_DECODE 0b00001110000000000000000000000000

#define COPROC_REG_ENCODE 0b00001111000000000000000000010000
#define COPROC_REG_DECODE 0b00001110000000000000000000010000

#define COPROC_LOAD_STORE_ENCODE 0b00001110000000000000000000000000
#define COPROC_LOAD_STORE_DECODE 0b00001100000000000000000000000000

#define SWI_ENCODE 0b00001111000000000000000000000000

#define IRQ_DISABLE(_reg) ((_reg & (1 << 7)) > 0)
#define FIQ_DISABLE(_reg) ((_reg & (1 << 6)) > 0)

// 1 for thumb state and 0 for arm state
#define STATE_BIT(_reg) ((_reg & (1 << 5)) > 0)

#define OP_CODE arm->data_bus

// TODO: default - unrecoverable state apply reset
#define SET_MODE(_reg, _mode)                                                  \
    switch (_reg & 0b11111) {                                                  \
    case 0b10000:                                                              \
        _mode = USR;                                                           \
    case 0b10001:                                                              \
        _mode = FIQ;                                                           \
    case 0b10010:                                                              \
        _mode = IRQ;                                                           \
    case 0b10011:                                                              \
        _mode = SVC;                                                           \
    case 0b10111:                                                              \
        _mode = ABT;                                                           \
    case 0b11011:                                                              \
        _mode = UND;                                                           \
    case 0b11111:                                                              \
        _mode = SYS;                                                           \
    default:                                                                   \
    }

static int execute_instruction;
int arm_exec(Arm *arm) {

    // static void *dp_inst_table[] = {&&DO_AND, &&DO_EOR, &&DO_SUB, &&DO_RSB,
    //                                 &&DO_ADD, &&DO_ADC, &&DO_SBC, &&DO_RSC,
    //                                 &&DO_TST, &&DO_TEQ, &&DO_CMP, &&DO_CMN,
    //                                 &&DO_ORR, &&DO_MOV, &&DO_BIC, &&DO_MVN};
    // static void *mult_inst_table[] = {
    //     &&DO_UND_MULTIPLY, &&DO_MUL,   &&DO_UND_MULTIPLY, &&DO_UND_MULTIPLY,
    //     &&DO_UMULL,        &&DO_UMLAL, &&DO_SMULL,        &&DO_SMLAL,
    //
    //
    // };
    //
    static void *cond_field_table[] = {
        &&CHECK_EQ, &&CHECK_NE, &&CHECK_CS_HS, 
        &&CHECK_CC_LO, &&CHECK_MI, &&CHECK_PL, 
        &&CHECK_VS, &&CHECK_VC, &&CHECK_HI, &&CHECK_LS,
        &&CHECK_GE, &&CHECK_LT, &&CHECK_GT, &&CHECK_LE, 
        &&CHECK_AL, &&UNCONDITIONAL
    };

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
    if ((OP_CODE & MULTIPLY_ENCODE) == MULTIPLY_DECODE) {
        goto MULTIPLY;
    } else if (((OP_CODE & LOAD_STORE_ENCODE) == LOAD_STORE_DECODE) &&
               ((OP_CODE & LOAD_STORE_ENCODE1) != 0)) {
        goto LOAD_STORE;
    } else if ((OP_CODE & CONTROL_DATA_ENCODE) == 0) {
        // instruction can be control and dsp instruction or data processing or
        // branch and exchange instruction

        if (((OP_CODE & CONTROL_DATA_ENCODE) == CONTROL_DSP_DECODE) &&
            ((OP_CODE & CONTORL_DSP_DECODE1) != CONTORL_DSP_DECODE1)) {
            goto CONTROL_DSP;
        } else if ((OP_CODE & BX_ENCODE) == BX_DECODE) {
            goto BRANCH_EXCHANGE;
        }
        goto DATA_PROCESSING;
    } else if ((OP_CODE & BBL_ENCODE) == BBL_DECODE) {
        goto BRANCH_LINK;
    } else if ((OP_CODE & LOAD_STORE_MULTIPLE_ENCODE) ==
               LOAD_STORE_MULTIPLE_DECODE) {
        goto LOAD_STORE_MULTIPLE;
    } else if ((OP_CODE & LOAD_STORE_IM_ENCODE) == LOAD_STORE_IM_DECODE) {
        goto LOAD_STORE_IM_OFFSET;
    } else if ((OP_CODE & LOAD_STORE_REG_ENCODE) == LOAD_STORE_REG_DECODE) {
        goto LOAD_STORE_REG_OFFSET;
    } else if ((OP_CODE & SWI_ENCODE) == SWI_ENCODE) {
        goto SOFTWARE_INTERRUPT;
    } else if ((OP_CODE & COPROC_LOAD_STORE_ENCODE) ==
               COPROC_LOAD_STORE_DECODE) {
        goto COPROCESSOR_LOAD_STORE;

    } else if ((OP_CODE & COPROC_DATA_ENCODE) == COPROC_DATA_DECODE) {
        goto COPROCESSOR_DATA;

    } else if ((OP_CODE & COPROC_REG_ENCODE) == COPROC_REG_DECODE) {
        goto COPROCESSOR_REGISTER;

    } else {
        // undefined instruction or media instructions
    }

    // A instruction can lie in following types

DATA_PROCESSING:
MULTIPLY:
CONTROL_DSP:
LOAD_STORE:
SOFTWARE_INTERRUPT:
LOAD_STORE_MULTIPLE:
LOAD_STORE_IM_OFFSET:
LOAD_STORE_REG_OFFSET:
BRANCH_LINK:
BRANCH_EXCHANGE:
COPROCESSOR_DATA:
COPROCESSOR_REGISTER:
COPROCESSOR_LOAD_STORE:
UNCONDITIONAL:

    return 0;
}
