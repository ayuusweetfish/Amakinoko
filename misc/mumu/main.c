#include "mumu.h"
#include <stdio.h>

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x300000ff, // addsn 0, 0, -1
    0x00000000, // sc 0
  };
  m.c = c;
  m.pc = 0;

  m.m[0] = 0xaa558800;
  printf("running!\n");
  mumu_run(&m);
  printf("%08x\n", m.m[0]); // 0xaa5587ff

  return 0;
}
