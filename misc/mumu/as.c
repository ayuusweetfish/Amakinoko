#include <ctype.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
  enum operand_type_t {
    REGISTER,
    IMMEDIATE,
    LABEL_B,
    LABEL_F,
    NONE,
  } ty;
  uint16_t n;
} operand_t;

typedef struct {
  uint8_t opcode;
  uint8_t n_operands;
  operand_t a, b, c;
} instruction_t;

operand_t read_operand()
{
  operand_t o;
  int c;

  // Find operand
  while ((c = getchar()) != EOF && c != '\n' && c != '%' && c != '=' && !isalpha(c)) { }
  if (c == EOF || c == '\n') return (operand_t){ .ty = NONE, .n = 0 };
  if (c == '%' || c == '=') {
    o.ty = (c == '%' ? REGISTER : IMMEDIATE);
    uint32_t n = 0;
    while ((c = getchar()) >= '0' && c <= '9') {
      n = n * 10 + (c - '0');
    }
    o.n = n;
    ungetc(c, stdin);
  } else if (isalpha(c)) {
    int dir = getchar();
    if (dir == 'b') {
      o.ty = LABEL_B;
    } else if (dir == 'f') {
      o.ty = LABEL_F;
    } else {
      printf("Invalid direction %c\n", dir);
      return (operand_t){ .ty = NONE, .n = 0 };
    }
    o.n = toupper(c) - 'A';
  }

  return o;
}

uint32_t assemble(uint32_t *restrict rom, uint32_t limit)
{
  uint32_t len = 0;
  int c;

  static const uint8_t mnemonic_trie[36][26] = {
  };

  while (1) {
    while ((c = getchar()) != EOF && isspace(c)) { }
    if (c == EOF) break;

    // Mnemonic
    uint8_t trie_pos = 0;
    putchar('[');
    do {
      putchar(c);
    } while ((c = getchar()) != EOF && isalpha(c));
    putchar(']'); putchar('\n');

    // Operands
    while (1) {
      operand_t o = read_operand();
      if (o.ty == NONE) break;
      printf("> operand %u %u\n", (unsigned)o.ty, (unsigned)o.n);
    }
  }

  return len;
}

int main()
{
  uint32_t c[256];
  uint32_t len = assemble(c, sizeof c / sizeof c[0]);

  for (uint32_t i = 0; i < len; i++)
    printf("%04x: %08x\n", i, c[i]);

  return 0;
}
