#include <stdio.h>
#include "mumu.h"

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x30010000, // movi 1, 0
    0x30020001, // movi 2, 1
    0x30040000, // movi 4, 0
                // A:
    0x10030101, // mov 3, 1
    0x10010202, // mov 1, 2
    0x11020103, // add 2, 1, 3
    0x32000001, // subi 0, 1
    0x910004fb, // brine 0, 4, Ab
    0x10000101, // mov 0, 1
    0x00000000, // sc 0
  };
  m.c = c;
  m.pc = 0;

  m.m[0] = 15;
  printf("running!\n");
  mumu_run(&m);
  printf("%08x\n", m.m[0]); // 0x00000262 = 610

  return 0;
}
