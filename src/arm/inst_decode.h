#ifndef SLIME_INST_DECODE_H
#define SLIME_INST_DECODE_H

#define NF_SET(_reg) ((_reg & (1 << 31)) > 0)

#define ZF_SET(_reg) ((_reg & (1 << 30)) > 0)

#define CF_SET(_reg) ((_reg & (1 << 29)) > 0)

#define VF_SET(_reg) ((_reg & (1 << 28)) > 0)

#define UNCONDITIONAL_ENCODE 0xF0000000

#define MULTIPLY_ENCODE 0xF0000F0
#define MULTIPLY_DECODE 0x90 

#define LOAD_STORE_ENCODE 0xE000090
#define LOAD_STORE_DECODE 0x90 
#define LOAD_STORE_ENCODE1 0x1000060 

#define LOAD_STORE_MULTIPLE_ENCODE 0xE000000 
#define LOAD_STORE_MULTIPLE_DECODE 0x8000000 

#define LOAD_STORE_IM_ENCODE 0xE000000 
#define LOAD_STORE_IM_DECODE 0x4000000 

#define LOAD_STORE_REG_ENCODE 0xE000010 
#define LOAD_STORE_REG_DECODE 0x6000000 

#define CONTROL_DATA_ENCODE 0xC000000 

#define CONTROL_DSP_ENCODE 0x1900000 
#define CONTROL_DSP_DECODE 0x1000000 

#define CONTROL_DSP_ENCODE1 0x1000090 
#define CONTORL_DSP_DECODE1 0x90 

#define BX_ENCODE 0xFF000F0 
#define BX_DECODE 0x1200010 
#define BBL_ENCODE 0xE000000 
#define BBL_DECODE 0xA000000 

#define COPROC_DATA_ENCODE 0xF000010 
#define COPROC_DATA_DECODE 0xE000000 

#define COPROC_REG_ENCODE 0xF000010 
#define COPROC_REG_DECODE 0xE000010 

#define COPROC_LOAD_STORE_ENCODE 0xE000000 
#define COPROC_LOAD_STORE_DECODE 0xC000000

#define SWI_ENCODE 0xF000000 

#define IRQ_DISABLE(_reg) ((_reg & (1 << 7)) > 0)
#define FIQ_DISABLE(_reg) ((_reg & (1 << 6)) > 0)

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
