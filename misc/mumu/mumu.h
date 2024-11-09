#pragma once

#include <stdint.h>
#ifdef MUMU_DEBUG
#include <stdio.h>
#endif

typedef struct {
  const uint32_t *c;
  uint32_t m[MUMU_RAM_SIZE];
  uint32_t pc;
} mumu_vm_t;

#pragma GCC push_options
#pragma GCC optimize("O3")
#if __arm__
__attribute__ ((section(".RamFunc")))
#endif
void mumu_run(mumu_vm_t *restrict m)
{
#define RAM(_n) m->m[(_n) & (MUMU_RAM_SIZE - 1)]
#define ROM(_n) m->c[(_n) & (MUMU_ROM_SIZE - 1)]

  uint32_t pc = m->pc;
  while (1) {
    uint32_t instr = m->c[(pc++) & (MUMU_ROM_SIZE - 1)];
  #ifdef MUMU_DEBUG
    printf("%08x | %08x %08x %08x %08x\n", instr, RAM(MUMU_DEBUG + 0), RAM(MUMU_DEBUG + 1), RAM(MUMU_DEBUG + 2), RAM(MUMU_DEBUG + 3));
  #endif
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
                    ((uint32_t)((instr >> 0) & 0xFFFF) << 16) // SDCC spurious warning workaround
      );
      uint32_t result = (
        op == 0x0 ? opnd_2 :
        op == 0x1 ? (opnd_1 + opnd_2) :
        op == 0x2 ? (opnd_1 - opnd_2) :
        op == 0x3 ? (opnd_1 * opnd_2) :
        op == 0x4 ? (opnd_1 & opnd_2) :
        op == 0x5 ? (opnd_1 | opnd_2) :
        op == 0x6 ? (opnd_1 ^ opnd_2) :
        op == 0x7 ? (opnd_1 & ~opnd_2) :
        op == 0x8 ? (opnd_1 >> (opnd_2 & 31)) :
        op == 0x9 ? (uint32_t)(((int32_t)opnd_1) >> (opnd_2 & 31)) :
        op == 0xA ? (opnd_1 << (opnd_2 & 31)) :
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

    case 0x8: case 0x9: case 0xA: case 0xB: {
      uint32_t opnd_1 = RAM((instr >> 16) & 0xFF);
      uint8_t b = (instr >> 8) & 0xFF;
      uint32_t opnd_2 = (ty == 0x9 ? (uint32_t)b : RAM(b));
      uint8_t c = (instr >> 0) & 0xFF;
      uint8_t op = opcode & 0xF;
      uint8_t result = (
        op == 0x0 ? 1 :
        op == 0x1 ? opnd_1 == opnd_2 :
        op == 0x2 ? opnd_1 != opnd_2 :
        op == 0x3 ? opnd_1 >  opnd_2 :
        op == 0x4 ? opnd_1 <  opnd_2 :
        op == 0x5 ? opnd_1 >= opnd_2 :
        op == 0x6 ? opnd_1 <= opnd_2 :
        op == 0x7 ? (int32_t)opnd_1 >  (int32_t)opnd_2 :
        op == 0x8 ? (int32_t)opnd_1 <  (int32_t)opnd_2 :
        op == 0x9 ? (int32_t)opnd_1 >= (int32_t)opnd_2 :
        op == 0xA ? (int32_t)opnd_1 <= (int32_t)opnd_2 :
                    0
      );
      if (result) {
        pc = (ty <= 0x9 ? pc + (int8_t)c :
              ty == 0xA ? pc + RAM(c) :
                          RAM(c));
      }
      break;
    }

    case 0xF: {
      if (opcode == 0xF0) {
        pc = (instr >> 0) & 0xFFFFFF;
      }
      break;
    }

    case 0xC: {
      uint32_t sp = RAM(MUMU_RAM_SIZE - 1);
      uint8_t a = (instr >> 16) & 0xFF;
      uint8_t b = (instr >>  8) & 0xFF;
      uint8_t i = (instr >>  0) & 0xFF;
      switch (opcode) {
      case 0xC0:  // push <A> <B>
        sp -= a; for (uint8_t i = a - 1; i != (uint8_t)-1; i--) RAM(sp + i) = RAM(b + i);
        break;
      case 0xC1:  // call <Imm24>
        RAM(--sp) = pc;
        pc = (instr >> 0) & 0xFFFFFF;
        break;
      // 0xC2, 0xC3, 0xC4 unused and untested
      case 0xC2:  // call <A> <B> <Imm8>
        RAM(--sp) = pc;
        sp -= a; for (uint8_t i = a - 1; i != (uint8_t)-1; i--) RAM(sp + i) = RAM(b + i);
        pc += (int8_t)i;
        break;
      case 0xC3:  // callf <A> <Imm16>
        RAM(--sp) = pc;
        sp -= a; for (uint8_t i = a - 1; i != (uint8_t)-1; i--) RAM(sp + i) = RAM(i);
        pc += (int16_t)((instr >> 0) & 0xFFFF);
        break;
      case 0xC4:  // calla <A> <B> <C>
        RAM(--sp) = pc;
        sp -= a; for (uint8_t i = a - 1; i != (uint8_t)-1; i--) RAM(sp + i) = RAM(i);
        pc = RAM(i);
        break;
      case 0xC8:  // pop <A> <B>
        for (uint8_t i = 0; i != a; i++) RAM(b + i) = RAM(sp + i); sp += a;
        break;
      case 0xC9:  // ret <A> <B>
        for (uint8_t i = 0; i != a; i++) RAM(b + i) = RAM(sp + i); sp += a;
        pc = RAM(sp++);
        break;
      default: break;
      }
      RAM(MUMU_RAM_SIZE - 1) = sp;
      break;
    }

    case 0x0: {
      uint32_t syscall = instr & (((uint32_t)1 << 24) - 1);
      if (syscall == 0) goto _fin;
      break;
    }

    default: break;
    }
  }

_fin:
  m->pc = pc;
}
#pragma GCC pop_options
