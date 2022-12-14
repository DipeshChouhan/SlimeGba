#ifndef SLIME_THUMB_INST_DECODE_H
#define SLIME_THUMB_INST_DECODE_H


#define COND_BRANCH_MASK 0xF000
#define COND_BRANCH_DECODE 0xD000

#define UNCOND_BRANCH_MASK 0xE000
#define UNCOND_BRANCH_DECODE 0xE000

#define BRANCH_EXCHANGE_MASK 0xFF00
#define BRANCH_EXCHANGE_DECODE 0x4700

#define DATA_PROCESS_F1_MASK 0xFC00
#define DATA_PROCESS_F1_DECODE 0x1800

#define DATA_PROCESS_F2_MASK 0xFC00
#define DATA_PROCESS_F2_DECODE 0x1C00

#define DATA_PROCESS_F3_MASK 0xE000
#define DATA_PROCESS_F3_DECODE 0x2000

#define DATA_PROCESS_F4_MASK 0xE000
#define DATA_PROCESS_F4_DECODE 0x0

#define DATA_PROCESS_F5_MASK 0xFC00
#define DATA_PROCESS_F5_DECODE 0x4000

#define DATA_PROCESS_F6_MASK 0xF000
#define DATA_PROCESS_F6_DECODE 0xA000
#define DATA_PROCESS_F7_MASK 0xFF00
#define DATA_PROCESS_F7_DECODE 0xB000

#define DATA_PROCESS_F8_MASK 0xFC00
#define DATA_PROCESS_F8_DECODE 0x4400

#define LOAD_STORE_F1_MASK 0xF800

#define LOAD_STORE_F2_MASK 0xFE00

#define LDR3_MASK 0xF800
#define LDR3_DECODE 0x4800

#define LOAD_STORE_F4_MASK 0xF000
#define LOAD_STORE_F4_DECODE 0x9000

#define LDMIA_DECODE 0xC800
#define STMIA_DECODE 0xC000
#define PUSH 0xB400
#define POP 0xBC00


#define LOAD_STORE_M_F2_MASK 0xF600
#define LOAD_STORE_M_F2_DECODE 0xB400

#define SWI_MASK 0xFF00
#define SWI_DECODE 0xDF00

#define LDR1_DECODE 0x6800
#define LDRB1_DECODE 0x7800
#define LDRH1_DECODE 0x8800
#define STR1_DECODE 0x6000
#define STRB1_DECODE 0x7000
#define STRH1_DECODE 0x8000

#define LDR2_DECODE 0x5800
#define LDRB2_DECODE 0x5C00
#define LDRH2_DECODE 0x5A00
#define LDRSB_DECODE 0x5600
#define LDRSH_DECODE 0x5E00
#define STR2_DECODE 0x5000
#define STRB2_DECODE 0x5400
#define STRH2_DECODE 0x5200

#define LDR4_DECODE 0x9800
#define STR3_DECODE 0x9000
#endif
