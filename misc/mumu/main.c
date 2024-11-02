#include <stdio.h>
#include <time.h>
#include "mumu.h"

int main()
{
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x30010000, // movi 1, 0
    0x30020001, // movi 2, 1
    0x30040001, // movi 4, 1
    0x30050000, // movi 5, 0
    0x86000404, // brile 0, 4, Bf
                // A:
    0x11010102, // add 1, 2
    0x11020201, // add 2, 1
    0x32000002, // subi 0, 2
    0x830004fc, // brigt 0, 4, Ab
                // B:
    0x81000502, // brie 0, 5, Cf
    0x10000202, // mov 0, 2
    0x00000000, // sc 0
                // C:
    0x10000101, // mov 0, 1
    0x00000000, // sc 0
  };
  m.c = c;

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

  for (int i = 1; i <= 16; i++) {
    m.m[0] = i;
    m.pc = 0;
    mumu_run(&m);
    printf("%2d %4u\n", i, m.m[0]);
  }

  return 0;
}
