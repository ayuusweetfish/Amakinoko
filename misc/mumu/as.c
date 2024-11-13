// gcc as.c mumu_as.c -DMUMU_AS_STDIO -o as -O2

#include "mumu_as.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static void print_usage_and_exit(const char *prog_name)
{
  fprintf(stderr, "Usage: %s [-c]\n", prog_name ? prog_name : "<prog-name>");
  fprintf(stderr, "  -c   Print instructions in C style (0x........)\n");
  exit(1);
}

int main(int argc, char *argv[])
{
  enum {
    FMT_DEFAULT,
    FMT_C,
  } format = FMT_DEFAULT;

  int opt;
  while ((opt = getopt(argc, argv, "ch")) != -1) {
    switch (opt) {
    case 'c':
      format = FMT_C;
      break;
    case 'h':
    default:
      print_usage_and_exit(argv[0]);
      break;
    }
  }

  mumu_as_init();

  uint32_t c[65536];
  struct file_pos_t err_pos;
  char err_msg[64];
  uint32_t len = mumu_as_assemble(
    stdin, c, sizeof c / sizeof c[0],
    &err_pos, err_msg, sizeof err_msg);

  if (format == FMT_DEFAULT) {
    for (uint32_t i = 0; i < len; i++) printf("%04" PRIx32 ": %08" PRIx32 "\n", i, c[i]);
  } else if (format == FMT_C) {
    for (uint32_t i = 0; i < len; i++) printf("    0x%08" PRIx32 ",\n", c[i]);
  }

  if (err_pos.line != 0)
    fprintf(stderr, "(%" PRIu32 ":%" PRIu32 "): %s\n", err_pos.line, err_pos.col, err_msg);

  return 0;
}
