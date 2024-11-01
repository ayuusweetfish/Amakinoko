#pragma once

#include <stdint.h>

#define MUMU_ROM_SIZE 1024
#define MUMU_MEM_SIZE 512

typedef struct {
  const uint32_t *c;
  uint32_t m[MUMU_MEM_SIZE];
  uint32_t pc;
} mumu_vm_t;

#if MUMU_SINGLETON
static mumu_vm_t mumu_instance;
#else
#endif

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__ ((section(".RamFunc")))
void mumu_run(
#if MUMU_SINGLETON
#else
  mumu_vm_t *m
#endif
) {
#if MUMU_SINGLETON
  mumu_vm_t *const m = &mumu_instance;
#else
#endif
  while (1) {
    uint32_t instr = m->c[(m->pc++) & (MUMU_ROM_SIZE - 1)];
    uint8_t opcode = instr >> 24;
    uint8_t ty = opcode >> 4;
    
    static const void *dispatch_ty[16] = {
      &&ty_0, &&ty_1, &&ty_2, &&ty_3,
      &&ty_4, &&ty_nop, &&ty_nop, &&ty_nop,
      &&ty_nop, &&ty_nop, &&ty_nop, &&ty_nop,
      &&ty_nop, &&ty_nop, &&ty_nop, &&ty_nop,
    };
    goto *dispatch_ty[ty];

    uint32_t opnd_1, opnd_2, result;
  ty_0:
    opnd_1 = instr & ((1 << 24) - 1);
    if (opnd_1 == 0) return;
    continue;
  ty_1:
    opnd_1 = m->m[(instr >> 8) & 0xFF];
    opnd_2 = m->m[(instr >> 0) & 0xFF];
    goto alu;
  ty_2:
    opnd_1 = m->m[(instr >> 8) & 0xFF];
    opnd_2 = ((instr >> 0) & 0xFF);
    goto alu;
  ty_3:
    opnd_1 = m->m[(instr >> 16) & 0xFF];
    opnd_2 = ((instr >> 0) & 0xFFFF);
    goto alu;
  ty_4:
    opnd_1 = m->m[(instr >> 16) & 0xFF];
    opnd_2 = (((instr >> 0) & 0xFFFF) << 16);
    goto alu;
  ty_nop:
    continue;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    uint8_t op;
  alu:
    op = opcode & 0xF;
    static const void *dispatch_op[16] = {
      &&op_0, &&op_1, &&op_2, &&op_3, &&op_4, &&op_5, &&op_6
    };
    goto *dispatch_op[op];
    do {
      op_0: result = (opnd_1 + opnd_2); break;
      op_1: result = (opnd_1 - opnd_2); break;
      op_2: result = (opnd_1 * opnd_2); break;
      op_3: result = (opnd_1 & opnd_2); break;
      op_4: result = (opnd_1 | opnd_2); break;
      op_5: result = (opnd_1 ^ opnd_2); break;
      op_6: result = (opnd_1 & ~opnd_2); break;
    } while (0);
#pragma GCC diagnostic pop

    m->m[(instr >> 16) & 0xFF] = result;
  }
}
#pragma GCC pop_options
