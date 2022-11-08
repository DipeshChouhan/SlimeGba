#ifndef SLIME_INST_DECODE_H
#define SLIME_INST_DECODE_H




#define MULTIPLY_MASK 0xF0000F0
#define MULTIPLY_DECODE 0x90
#define DATA_PROCESS_MASK 0xC000000
#define DATA_PROCESS_DECODE 0x0

#define CONTROL_MASK 0x1900000
#define CONTROL_DECODE 0x1000000

#define LOAD_STORE_H_D_S_MASK 0xE000090
#define LOAD_STORE_H_D_S_DECODE 0x90

#define LOAD_STORE_W_U_MASK 0xC000000
#define LOAD_STORE_W_U_DECODE 0x4000000

#define LOAD_STORE_M_MASK 0xE000000
#define LOAD_STORE_M_DECODE 0x8000000

#define BRANCH_LINK_MASK 0xE000000
#define BRANCH_LINK_DECODE 0xA000000

#define SWI_MASK 0xF000000
#define SWI_DECODE 0xF000000

#define COPROCESSOR_MASK 0xC000000
#define COPROCESSOR_DECODE 0xC000000

#define SHIFTER_IMM_MASK 0x2000000
#define SHIFTER_IMM_DECODE 0x2000000

#define SHIFTER_REG_MASK 0x2000FF0
#define SHIFTER_REG_DECODE 0x0

#define SHIFTER_SHIFT_IMM_MASK 0x2000070
#define SHIFTER_LSL_IMM_MASK 0x2000070
#define SHIFTER_LSL_IMM_DECODE 0x0

#define SHIFTER_SHIFT_REG_MASK 0x20000F0
#define SHIFTER_LSL_REG_MASK 0x20000F0
#define SHIFTER_LSL_REG_DECODE 0x10

#define SHIFTER_LSR_IMM_MASK 0x2000070
#define SHIFTER_LSR_IMM_DECODE 0x20

#define SHIFTER_LSR_REG_MASK 0x20000F0
#define SHIFTER_LSR_REG_DECODE 0x30

#define SHIFTER_ASR_IMM_MASK 0x2000070
#define SHIFTER_ASR_IMM_DECODE 0x40

#define SHIFTER_ASR_REG_MASK 0x20000F0
#define SHIFTER_ASR_REG_DECODE 0x50

#define SHIFTER_ROR_IMM_MASK 0x2000060
#define SHIFTER_ROR_IMM_DECODE 0x60

#define SHIFTER_ROR_REG_MASK 0x20000F0
#define SHIFTER_ROR_REG_DECODE 0x70

#define SHIFTER_ROR_EXTEND_MASK 0x2000FF0
#define SHIFTER_ROR_EXTEND_DECODE 0x60


#define LS_W_U_IMM_MASK 0x3200000
#define LS_W_U_SCALED_MASK 0x3200010
#define LS_W_U_IMM_DECODE 0x1000000

#define LS_W_U_REG_MASK 0x3200FF0 
#define LS_W_U_REG_DECODE 0x3000000

#define LS_W_U_SCALED_REG_MASK 0x3200010
#define LS_W_U_SCALED_REG_DECODE 0x3000000

#define LS_W_U_IMM_PR_MASK 0x3200000
#define LS_W_U_IMM_PR_DECODE 0x1200000

#define LS_W_U_REG_PR_MASK 0x3200FF0
#define LS_W_U_REG_PR_DECODE 0x3200000

#define LS_W_U_SCALED_REG_PR_MASK 0x3200010
#define LS_W_U_SCALED_REG_PR_DECODE 0x3200000

#define LS_W_U_IMM_PO_MASK 0x3000000
#define LS_W_U_IMM_PO_DECODE 0x0

#define LS_W_U_REG_PO_MASK 0x3000FF0
#define LS_W_U_REG_PO_DECODE 0x2000000

#define LS_W_U_SCALED_REG_PO_MASK 0x3000010
#define LS_W_U_SCALED_REG_PO_DECODE 0x2000000

#define LS_H_D_S_MASK 0x1600000

#define LS_H_D_S_IMM_DECODE 0x1400000
#define LS_H_D_S_REG_DECODE 0x1000000
#define LS_H_D_S_IMM_PR_DECODE 0x1600000
#define LS_H_D_S_REG_PR_DECODE 0x1200000
#define LS_H_D_S_IMM_PO_DECODE 0x400000
#define LS_H_D_S_REG_PO_DECODE 0x0

#define LS_M_MASK 0x1800000

#define LS_M_IA_DECODE 0x800000
#define LS_M_IB_DECODE 0x1800000
#define LS_M_DA_DECODE 0x0
#define LS_M_DB_DECODE 0x1000000

#define LDR_DECODE 0x100000
#define LDRB_DECODE 0x500000
#define STR_DECODE 0x0
#define STRB_DECODE 0x400000

#define LDRBT_DECODE 0x700000
#define LDRT_DECODE 0x300000

#define STRBT_DECODE 0x600000
#define STRT_DECODE 0x200000

#define BX_MASK 0x6000F0
#define BX_DECODE 0x200010

#define MRS_MASK 0x200000
#define MRS_DECODE 0x0

#define MSR_IMM_MASK 0x2200000
#define MSR_IMM_DECODE 0x2200000

#define MSR_REG_MASK 0x2000F0
#define MSR_REG_DECODE 0x200000

#define LDM1_MASK 0x500000
#define LDM1_DECODE 0x100000

#define LDM2_MASK 0x708000
#define LDM2_DECODE 0x500000

#define LDM3_MASK 0x508000
#define LDM3_DECODE 0x508000

#define STM1_MASK 0x500000
#define STM1_DECODE 0x0

#define STM2_MASK 0x700000
#define STM2_DECODE 0x400000



// 1 for thumb state and 0 for arm state
#define STATE_BIT(_reg) ((_reg & (1 << 5)) > 0)

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
#endif
