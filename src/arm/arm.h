#ifndef SLIME_ARM_H
#define SLIME_ARM_H
#include <stdint.h>
// CAUTION: Do not change the sequence in Mode enum
typedef enum { USR = 0, // user
  SYS,     // system
  IRQ,     // interrupt
  SVC,     // supervisor
  ABT,     // abort
  UND,     // undefined
  FIQ,     // fast interrupt
} Mode;

typedef struct Arm {
  // SysBus sys_bus;
  int state; // 1 for thumb and 0 for arm state
  Mode mode; // operation mode

  uint32_t curr_instruction; // address
  int shifter_carry_out;     // internal
  uint32_t data_bus;         // 32 bit data bus
  uint32_t address_bus;

  // r15 is program counter register
  uint32_t general_regs[16]; // general purpose registers r0 - r15. User and
                             // System mode

  uint32_t fiq_regs[7]; // banked registers from r8 - r14
  uint32_t svc_regs[2]; // banked registers from r13 - r14
  uint32_t abt_regs[2]; // banked registers from r13 - r14
  uint32_t irq_regs[2]; // banked registers from r13 - r14
  uint32_t und_regs[2]; // banked registers from r13 - r14

  uint32_t *reg_table[116];
  // ARM-state program status registers
  uint32_t cpsr;
  // don't change order below
  union {
    uint32_t spsr[5];
    uint32_t spsr_fiq;
    uint32_t spsr_irq;
    uint32_t spsr_svc;
    uint32_t spsr_abt;
    uint32_t spsr_und;
  };

  // exception
  int exception_gen; // shows an exception generated externally
  int reset_pin;
  int fiq_pin;
  int irq_pin;

} Arm;

#define RESET_LOW_VECTOR 0x00000000
#define UNDEFINED_LOW_VECTOR 0x00000004
#define SWI_LOW_VECTOR 0x00000008
#define P_ABORT_LOW_VECTOR 0x0000000C
#define DATA_ABORT_LOW_VECTOR 0x00000010
#define IRQ_LOW_VECTOR 0x00000018
#define FIQ_LOW_VECTOR 0x0000001C

#define USER_MODE 0b10000
#define FIQ_MODE 0b10001
#define IRQ_MODE 0b10010
#define SVC_MODE 0b10011
#define ABORT_MODE 0b10111
#define UND_MODE 0b11011
#define SYS_MODE 0b11111

#define R14_SVC 1
#define R14_FIQ 6
#define R14_IRQ 1

#define ARM_STATE 0
#define THUMB_STATE 1
#define NF_BIT 31
#define ZF_BIT 30
#define CF_BIT 29
#define VF_BIT 28

#define IS_BIT_SET(_op, _bit) ((_op & (1 << _bit)) == (1 << _bit))

#define IS_BIT_NOT_SET(_op, _bit) ((_op & (1 << _bit)) != (1 << _bit))
#define ROTATE_RIGHT32(_op, _ror) ((_op >> _ror) | (_op << (32 - _ror)))

#define GET_BIT(_op, _bit) ((_op >> _bit) & 1)

#define MEM_WRITE(_address, _data, _type)                                      \
  ((Gba *)arm)->memory.address_bus = _address;                                 \
  ((Gba *)arm)->memory.data_bus = _data;                                       \
  _type(&((Gba *)arm)->memory);

#define MEM_READ(_address, _dest, _type)                                       \
  ((Gba *)arm)->memory.address_bus = _address;                                 \
  _dest = _type(&((Gba *)arm)->memory);

#define SIGN_EXTEND(_value, _bit) (_value | (IS_BIT_SET(_value, _bit) * (0xFFFFFFFF << (_bit + 1))));

#define SET_BIT(_op, _bit, _to) ((_op & (~(1 << _bit))) | (_to << _bit))

#define SET_P_MODE(_cpsr, _mode) _cpsr = (_cpsr & 0x1F) | _mode;

#define SET_P_STATE(_cpsr, _state) _cpsr = SET_BIT(_cpsr, 5, _state);

#define DISABLE_IRQ(_cpsr) _cpsr = SET_BIT(_cpsr, 7, 1);
#define DISABLE_FIQ(_cpsr) _cpsr = SET_BIT(_cpsr, 6, 1);

#define SWI_INSTRUCTION(_arm)                                                  \
  _arm->svc_regs[R14_SVC] = _arm->curr_instruction;                            \
  _arm->spsr_svc = _arm->cpsr;                                                 \
  SET_P_MODE(_arm->cpsr, SVC_MODE);                                            \
  _arm->mode = SVC;                                                            \
  SET_P_STATE(_arm->cpsr, ARM_STATE);                                          \
  _arm->state = ARM_STATE;                                                     \
  DISABLE_IRQ(_arm->cpsr);                                                     \
  _arm->curr_instruction = SWI_LOW_VECTOR;

#define INTERRUPT_REQUEST()                                                    \
  if (arm->exception_gen) {                                                    \
    arm->exception_gen = 0;                                                    \
    if (arm->reset_pin) {                                                      \
      arm->reset_pin = 0;                                                      \
      arm->fiq_pin = 0;                                                        \
      arm->irq_pin = 0;                                                        \
      SET_P_MODE(arm->cpsr, SVC_MODE);                                         \
      arm->mode = SVC;                                                         \
      SET_P_STATE(arm->cpsr, ARM_STATE);                                       \
      arm->state = ARM_STATE;                                                  \
      DISABLE_IRQ(arm->cpsr);                                                  \
      DISABLE_FIQ(arm->cpsr);                                                  \
      arm->curr_instruction = RESET_LOW_VECTOR;                                \
    } else if (arm->fiq_pin && IS_BIT_SET(arm->cpsr, 6)) {                     \
      arm->fiq_pin = 0;                                                        \
      arm->fiq_regs[R14_FIQ] = arm->curr_instruction + 4;                      \
      arm->spsr_fiq = arm->cpsr;                                               \
      SET_P_MODE(arm->cpsr, FIQ_MODE);                                         \
      arm->mode = FIQ;                                                         \
      SET_P_STATE(arm->cpsr, ARM_STATE);                                       \
      arm->state = ARM_STATE;                                                  \
      DISABLE_FIQ(arm->cpsr);                                                  \
      DISABLE_IRQ(arm->cpsr);                                                  \
      arm->curr_instruction = FIQ_LOW_VECTOR;                                  \
    } else if (arm->irq_pin && IS_BIT_SET(arm->cpsr, 7)) {                     \
      arm->irq_pin = 0;                                                        \
      arm->irq_regs[R14_IRQ] = arm->curr_instruction + 4;                      \
      arm->spsr_irq = arm->cpsr;                                               \
      SET_P_MODE(arm->cpsr, IRQ_MODE);                                         \
      arm->mode = IRQ;                                                         \
      SET_P_STATE(arm->cpsr, ARM_STATE);                                       \
      arm->state = ARM_STATE;                                                  \
      DISABLE_IRQ(arm->cpsr);                                                  \
      arm->curr_instruction = IRQ_LOW_VECTOR;                                  \
    }                                                                          \
  }

void init_arm(Arm *arm);
// execute a single arm mode instruction and returns cycle count
int arm_exec(Arm *arm);
int thumb_exec(Arm *arm);
#endif /* ! */
