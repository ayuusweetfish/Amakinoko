#pragma once

#include <stdint.h>

#define UXN_RAM_SIZE 2048

typedef struct {
  uint8_t dat[0x100], ptr;
} uxn_stack_t;

typedef struct {
  uint8_t ram[UXN_RAM_SIZE];
  uxn_stack_t wst, rst;
} uxn_t;

uxn_t *uxn_instance();
int uxn_eval(uint16_t pc);
