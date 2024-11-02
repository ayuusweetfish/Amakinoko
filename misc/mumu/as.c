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

enum mnemonic_word {
  MN_WORD_BASE = 0x80,
  MN_MOV, MN_ADD, MN_SUB, MN_MUL,
  MN_AND, MN_OR, MN_XOR, MN_BIC, MN_LSR, MN_ASR, MN_LSL,
  MN_LD, MN_ST, MN_LDC, MN_LDCR,
  MN_BR, MN_BA,
  MN_SC,
};
static uint8_t mnemonic_trie[36][27];

static void init_trie()
{
  static struct {
    const char *w;
    enum mnemonic_word id;
  } mnemonics[] = {
    {"mov", MN_MOV},
    {"add", MN_ADD},
    {"sub", MN_SUB},
    {"mul", MN_MUL},
    {"and", MN_AND},
    {"or", MN_OR},
    {"xor", MN_XOR},
    {"bic", MN_BIC},
    {"lsr", MN_LSR},
    {"asr", MN_ASR},
    {"lsl", MN_LSL},
    {"ld", MN_LD},
    {"st", MN_ST},
    {"ldc", MN_LDC},
    {"ldcr", MN_LDCR},
    {"br", MN_BR},
    {"ba", MN_BA},
    {"sc", MN_SC},
  };

  uint8_t trie_size = 1;
  for (uint8_t i = 0; i < sizeof mnemonics / sizeof mnemonics[0]; i++) {
    uint8_t p = 0;
    for (const char *w = mnemonics[i].w; *w != 0; w++) {
      uint8_t q = mnemonic_trie[p][*w - 'a' + 1];
      if (q == 0)
        q = mnemonic_trie[p][*w - 'a' + 1] = trie_size++;
      p = q;
      printf("%c %d\n", *w, p);
    }
    mnemonic_trie[p][0] = mnemonics[i].id;
  }
}

uint32_t assemble(uint32_t *restrict rom, uint32_t limit)
{
  uint32_t len = 0;
  int c;

  while (1) {
    while ((c = getchar()) != EOF && isspace(c)) { }
    if (c == EOF) break;

    // Mnemonic
    uint8_t trie_pos = 0;
    putchar('[');
    do {
      c = tolower(c);
      putchar(c);
      trie_pos = mnemonic_trie[trie_pos][c - 'a' + 1];
      if (trie_pos == 0) {
        printf("Invalid mnemonic\n");
      }
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
  init_trie();

  uint32_t c[256];
  uint32_t len = assemble(c, sizeof c / sizeof c[0]);

  for (uint32_t i = 0; i < len; i++)
    printf("%04x: %08x\n", i, c[i]);

  return 0;
}
