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

  MN_B_BASE = 0x40,
  MN_B_AL, MN_B_EQ, MN_B_NE, MN_B_GT, MN_B_LT, MN_B_GE, MN_B_LE,
  MN_B_SGT, MN_B_SLT, MN_B_SGE, MN_B_SLE,
};
static uint8_t mnemonic_trie[73][27];

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
  }, conditions[] = {
    {"al", MN_B_AL}, 
    {"e", MN_B_EQ}, 
    {"eq", MN_B_EQ}, 
    {"ne", MN_B_NE}, 
    {"gt", MN_B_GT}, 
    {"lt", MN_B_LT}, 
    {"ge", MN_B_GE}, 
    {"le", MN_B_LE}, 
    {"sgt", MN_B_SGT}, 
    {"slt", MN_B_SLT}, 
    {"sge", MN_B_SGE}, 
    {"sle", MN_B_SLE},
  };

  uint8_t trie_size = 1;
  for (uint8_t i = 0; i < sizeof mnemonics / sizeof mnemonics[0]; i++) {
    uint8_t p = 0;
    for (const char *w = mnemonics[i].w; *w != 0; w++) {
      uint8_t q = mnemonic_trie[p][*w - 'a' + 1];
      if (q == 0)
        q = mnemonic_trie[p][*w - 'a' + 1] = trie_size++;
      p = q;
    }
    if (mnemonics[i].id == MN_BR || mnemonics[i].id == MN_BA) {
      // Condition suffices
      uint8_t p0 = p;
      for (uint8_t j = 0; j < sizeof conditions / sizeof conditions[0]; j++) {
        uint8_t p = p0;
        for (const char *w = conditions[j].w; *w != 0; w++) {
          uint8_t q = mnemonic_trie[p][*w - 'a' + 1];
          if (q == 0)
            q = mnemonic_trie[p][*w - 'a' + 1] = trie_size++;
          p = q;
        }
        mnemonic_trie[p][0] = mnemonics[i].id | conditions[j].id;
      }
    } else {
      mnemonic_trie[p][0] = mnemonics[i].id;
    }
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
    uint8_t mnemonic_valid = 1; // Cleared when a non-existent child is encountered
    uint8_t mnemonic = 0;       // Word tag
    uint8_t label_id = 0, len = 0;
    do {
      c = tolower(c);
      trie_pos = mnemonic_trie[trie_pos][c - 'a' + 1];
      if (++len == 1) label_id = c - 'a';
      if (trie_pos == 0) {
        mnemonic_valid = 0;
      }
    } while ((c = getchar()) != EOF && isalpha(c));
    if (c == ':') {
      if (len != 1) {
        printf("Invalid label\n");
      } else {
        printf("label %u\n", label_id);
      }
    } else {
      if (!mnemonic_valid || (mnemonic = mnemonic_trie[trie_pos][0]) == 0) {
        printf("Invalid mnemonic\n");
      }
    }

    // Operands
    if (mnemonic != 0) printf("Mnemonic %02x\n", mnemonic);
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
