#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
#include "mumu.h"

#include <stdio.h>
#include <time.h>

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x91100102,
    0x30100001,
    0xc100000b,
    0x310e0001,
    0x920e6405,
    0x400e0000,
    0x30080100,
    0x70090800,
    0x3609ffff,
    0x71090800,
    0x00000000,
    0x40090000,
    0x300a0100,
    0x300bccff,
    0x410b0066,
    0x110c090a,
    0x710b0c00,
    0x31090001,
    0x940918fc,
    0xc9000000,
  };
  m.c = c;
  m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;

  for (int i = 0; i < 5; i++) {
    m.pc = 0;
    mumu_run(&m);
    printf("%d\n", i);
    for (int j = 0; j < 24; j++)
      printf("%08x%c", m.m[128 + j], j == 23 ? '\n' : ' ');
  }

  return 0;
}

