#ifndef SLIME_ARM_H 
#define SLIME_ARM_H
#include <stdint.h>

typedef enum {
    USR, // user
    FIQ, // fast interrupt
    IRQ, // interrupt
    SVC, // supervisor
    ABT, // abort
    SYS, // system
    UND, // undefined
} Mode;

typedef struct Arm {
    // SysBus sys_bus;
    int state; // 1 for thumb and 0 for arm state
    Mode mode; // operation mode

    uint32_t data_bus;         // 32 bit data bus
    uint32_t general_regs[15]; // general purpose registers r0 - r14. User and
                               // System mode

    uint32_t r15;         // program counter register unbanked
    uint32_t fiq_regs[7]; // banked registers from r8 - r14
    uint32_t svc_regs[2]; // banked registers from r13 - r14
    uint32_t abt_regs[2]; // banked registers from r13 - r14
    uint32_t irq_regs[2]; // banked registers from r13 - r14
    uint32_t und_regs[2]; // banked registers from r13 - r14

    // ARM-state program status registers
    uint32_t cpsr;
    uint32_t spsr_fiq;
    uint32_t spsr_svc;
    uint32_t spsr_abt;
    uint32_t spsr_irq;
    uint32_t spsr_und;
} Arm;

// execute a single instruction and returns cycle count
int arm_exec(Arm *arm);
#endif /* ! */
