#pragma once

#include <stdint.h>

#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512

typedef struct {
  const uint32_t *c;
  uint32_t m[MUMU_RAM_SIZE];
  uint32_t pc;
} mumu_vm_t;

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__ ((section(".RamFunc")))
void mumu_run(mumu_vm_t *restrict m)
{
#define RAM(_n) m->m[(_n) & (MUMU_RAM_SIZE - 1)]
#define ROM(_n) m->c[(_n) & (MUMU_ROM_SIZE - 1)]

  uint32_t pc = m->pc;
  while (1) {
    uint32_t instr = m->c[(pc++) & (MUMU_ROM_SIZE - 1)];
    uint8_t opcode = instr >> 24;
    uint8_t ty = opcode >> 4;
    switch (ty) {
    case 0x1: case 0x2: case 0x3: case 0x4: {
      uint8_t op = opcode & 0xF;
      uint32_t opnd_1 = (ty <= 2 ? m->m[(instr >> 8) & 0xFF] : m->m[(instr >> 16) & 0xFF]);
      uint32_t opnd_2 = (
        ty == 0x1 ? m->m[(instr >> 0) & 0xFF] :
        ty == 0x2 ? ((instr >> 0) & 0xFF) :
        ty == 0x3 ? ((instr >> 0) & 0xFFFF) :
                    (((instr >> 0) & 0xFFFF) << 16)
      );
      uint32_t result = (
        op == 0x0 ? (opnd_1 + opnd_2) :
        op == 0x1 ? (opnd_1 - opnd_2) :
        op == 0x2 ? (opnd_1 * opnd_2) :
        op == 0x3 ? (opnd_1 & opnd_2) :
        op == 0x4 ? (opnd_1 | opnd_2) :
        op == 0x5 ? (opnd_1 ^ opnd_2) :
        op == 0x6 ? (opnd_1 & ~opnd_2) :
                    0
      );
      m->m[(instr >> 16) & 0xFF] = result;
      break;
    }

    case 0x7: {
      uint8_t a   = (instr >> 16) & 0xFF;
      uint8_t b   = (instr >>  8) & 0xFF;
      uint8_t imm = (instr >>  0) & 0xFF;
      switch (opcode & 0xF) {
        case 0: RAM(a) = RAM(RAM(b) + imm); break;
        case 1: RAM(RAM(b) + imm) = RAM(a); break;
        case 2: RAM(a) = ROM(RAM(b) + imm); break;
        case 3: RAM(a) = ROM(pc + RAM(b) + imm); break;
        default: break;
      }
      break;
    }

    case 0x8: case 0x9: case 0xA: {
      uint32_t opnd_1 = RAM((instr >> 16) & 0xFF);
      uint32_t opnd_2 = RAM((instr >>  8) & 0xFF);
      uint8_t c = (instr >> 0) & 0xFF;
      uint8_t op = opcode & 0xF;
      uint8_t result = (
        op == 0x0 ? opnd_1 == opnd_2 :
        op == 0x1 ? opnd_1 != opnd_2 :
        op == 0x2 ? opnd_1 >  opnd_2 :
        op == 0x3 ? opnd_1 <  opnd_2 :
        op == 0x4 ? opnd_1 >= opnd_2 :
        op == 0x5 ? opnd_1 <= opnd_2 :
        op == 0x6 ? (int32_t)opnd_1 >  (int32_t)opnd_2 :
        op == 0x7 ? (int32_t)opnd_1 <  (int32_t)opnd_2 :
        op == 0x8 ? (int32_t)opnd_1 >= (int32_t)opnd_2 :
        op == 0x9 ? (int32_t)opnd_1 <= (int32_t)opnd_2 :
                    0
      );
      if (result) {
        pc = (ty == 0x8 ? pc + RAM(c) :
              ty == 0x9 ? pc + c :
                          RAM(c));
      }
    }

    case 0x0: {
      uint32_t syscall = instr & ((1 << 24) - 1);
      if (syscall == 0) goto _fin;
    }

    default: break;
    }
  }

_fin:
  m->pc = pc;
}
#pragma GCC pop_options
