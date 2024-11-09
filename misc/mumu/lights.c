#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
#include "mumu.h"

#include <stdio.h>
#include <time.h>

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x30080040,
    0x40090000,
    0x300a0018,
    0x300bccff,
    0x450b0066,
    0x110c0809,
    0x710b0c00,
    0x31090001,
    0x84090afc,
    0x00000000,
  };
  m.c = c;
  m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;

  for (int i = 0; i < 3; i++) {
    m.pc = 0;
    mumu_run(&m);
    printf("%d\n", i);
    for (int j = 0; j < 24; j++)
      printf("  %08x\n", m.m[64 + j]);
  }

  return 0;
}

