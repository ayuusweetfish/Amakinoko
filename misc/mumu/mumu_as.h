#pragma once

#include <stdint.h>

void mumu_as_init();

struct file_pos_t {
  uint32_t line, col;
};

uint32_t mumu_as_assemble(
  uint32_t *restrict rom, uint32_t limit,
  struct file_pos_t *o_err_pos, char *o_err_msg, uint32_t err_msg_len_limit);
