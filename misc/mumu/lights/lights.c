// gcc lights.c -I.. -O2 -o lights
// ../as -b < 1.mumu | ./lights

#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
#include "mumu.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
  FILE *f;
  if (argc < 2) {
    f = stdin;
  } else {
    f = fopen(argv[1], "rb");
    if (f == NULL) {
      printf("Cannot open file %s\n", argv[1]);
      return 1;
    }
  }

  static mumu_vm_t m;
  static uint32_t c[MUMU_ROM_SIZE];

  int len;
  for (len = 0; len < MUMU_ROM_SIZE; len++) {
    uint32_t w = 0;
    for (int i = 0; i < 4; i++) {
      int c = fgetc(f);
      if (c == EOF) {
        if (i == 0) goto fin_rom;
        else {
          printf("ROM size must be a multiple of 4 bytes (32 bits); actual %d\n", len);
          return 1;
        }
      }
      w |= ((uint32_t)(uint8_t)c) << (i * 8);
    }
    c[len] = w;
  }
fin_rom:
  if (len == 0) {
    printf("ROM empty?\n");
    return 1;
  }
  if (len == MUMU_ROM_SIZE && !feof(f)) {
    printf("ROM size limit exceeded; limit %d words (%d bytes)\n", MUMU_ROM_SIZE, MUMU_ROM_SIZE * 4);
    return 1;
  }
  if (ferror(f) != 0) {
    printf("Cannot read from file %s\n", argv[1]);
    return 1;
  }
  fclose(f);

  memset(c + len, 0xff, sizeof c - len * sizeof c[0]);

  m.c = c;
  m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;

  for (int i = 0; i < 200; i++) {
    m.pc = 0;
    mumu_run(&m);
    printf("%3d | ", i);
    for (int j = 0; j < 24; j++)
      printf("%06x%c", m.m[256 + j] & 0xffffff, j == 23 ? '\n' : ' ');
    if (m.m[MUMU_RAM_SIZE - 1] != MUMU_RAM_SIZE - 1) printf("*** Stack residue? ***\n");
  }

  return 0;
}

