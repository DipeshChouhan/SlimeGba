#ifndef SLIME_INST_DECODE_H
#define SLIME_INST_DECODE_H

#define NF_SET(_reg) ((_reg & (1 << 31)) > 0)

#define ZF_SET(_reg) ((_reg & (1 << 30)) > 0)

#define CF_SET(_reg) ((_reg & (1 << 29)) > 0)

#define VF_SET(_reg) ((_reg & (1 << 28)) > 0)


#define IRQ_DISABLE(_reg) ((_reg & (1 << 7)) > 0)
#define FIQ_DISABLE(_reg) ((_reg & (1 << 6)) > 0)

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

#define SWP_MASK 0x4000F0
#define SWP_DECODE 0x90

#define SWPB_MASK 0x4000F0
#define SWPB_DECODE 0x400090
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
