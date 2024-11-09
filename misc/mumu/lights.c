#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
#include "mumu.h"

#include <stdio.h>
#include <time.h>

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x300f0001,
    0x81100f02,
    0x1010100f,
    0xc100000d,
    0x310e0001,
    0x300f0002,
    0x820e0f05,
    0x400e0000,
    0x30080040,
    0x70090800,
    0x3609ffff,
    0x71090800,
    0x00000000,
    0x30080040,
    0x40090000,
    0x300a0018,
    0x300bccff,
    0x410b0066,
    0x110c0809,
    0x710b0c00,
    0x31090001,
    0x84090afc,
    0xc9000000,
  };
  m.c = c;
  m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;

  for (int i = 0; i < 5; i++) {
    m.pc = 0;
    mumu_run(&m);
    printf("%d\n", i);
    for (int j = 0; j < 24; j++)
      printf("%08x%c", m.m[64 + j], j == 23 ? '\n' : ' ');
  }

  return 0;
}

