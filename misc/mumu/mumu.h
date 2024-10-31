#pragma once

#include <stdint.h>

#define MUMU_ROM_SIZE 1024
#define MUMU_MEM_SIZE 1024

typedef struct {
  const uint32_t *c;
  uint32_t m[MUMU_MEM_SIZE];
  uint32_t pc;
} mumu_vm_t;

#pragma GCC push_options
#pragma GCC optimize("O3")
void mumu_run(mumu_vm_t *m)
{
  while (1) {
    uint32_t instr = m->c[(m->pc++) & (MUMU_ROM_SIZE - 1)];
    uint8_t opcode = instr >> 24;
    if (opcode >= 0x10 && opcode <= 0x6F) {
      uint8_t op = opcode & 0xF;
      uint32_t opnd_1 = (opcode <= 0x3F ? m->m[(instr >> 8) & 0xFF] : m->m[(instr >> 16) & 0xFF]);
      uint32_t opnd_2 = (
        opcode <= 0x1F ? m->m[(instr >> 0) & 0xFF] :
        opcode <= 0x2F ? ((instr >> 0) & 0xFF) :
        opcode <= 0x3F ? (uint32_t)(int32_t)(int8_t)((instr >> 0) & 0xFF) :
        opcode <= 0x4F ? ((instr >> 0) & 0xFFFF) :
        opcode <= 0x5F ? (uint32_t)(int32_t)(int16_t)((instr >> 0) & 0xFFFF) :
                         (((instr >> 0) & 0xFFFF) << 16)
      );
      uint32_t result = (
        op == 0x0 ? (opnd_1 + opnd_2) :
        op == 0x1 ? (opnd_1 + opnd_2) :
        op == 0x2 ? (opnd_1 * opnd_2) :
        op == 0x3 ? (opnd_1 & opnd_2) :
        op == 0x4 ? (opnd_1 | opnd_2) :
        op == 0x5 ? (opnd_1 ^ opnd_2) :
        op == 0x6 ? (opnd_1 & ~opnd_2) :
                    0
      );
      m->m[(instr >> 16) & 0xFF] = result;
    } else if (opcode == 0x00) {
      uint32_t syscall = opcode & ((1 << 24) - 1);
      if (syscall == 0) break;
    }
  }
}
#pragma GCC pop_options
