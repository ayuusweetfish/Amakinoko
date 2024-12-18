#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
#include "mumu.h"

#include <stdio.h>
#include <time.h>

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x40010000,
    0x30020001,
    0x30040001,
    0x40050000,
    0x86000404,
    0x11010102,
    0x11020201,
    0x32000002,
    0x830004fc,
    0x81000502,
    0x10000002,
    0x00000000,
    0x10000001,
    0x00000000,
  };
  m.c = c;
  m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;

if (0) {
  for (int i = 0; i < 10; i++) {
    m.m[0] = 60;
    m.pc = 0;
    mumu_run(&m);
  }
  clock_t t0 = clock();
  for (int i = 0; i < 1000000; i++) {
    m.m[0] = 60;
    m.pc = 0;
    mumu_run(&m);
  }
  float t = (double)(clock() - t0) / CLOCKS_PER_SEC;
  printf("%.5lf %.5lf\n", t, 128 * 1000000 / t);  // ~68M instructions per second
}

  for (int i = 1; i <= 16; i++) {
    m.m[0] = i;
    m.pc = 0;
    mumu_run(&m);
    printf("%2d %4u %08x\n", i, m.m[0], m.m[1]);
  }

  return 0;
}
