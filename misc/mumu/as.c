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
  uint32_t n;
} operand_t;

typedef struct {
  uint8_t opcode;
  uint8_t n_operands;
  operand_t a, b, c;
} instruction_t;

static uint32_t line_num; // TODO: Refactor this, restricting `read_operand()` in the scope of `assemble()`

static inline int my_getchar()
{
  int c = getchar();
  if (c == '\n') line_num++;
  return c;
}
static inline void my_ungetchar(int c)
{
  ungetc(c, stdin);
  if (c == '\n') line_num--;
}

operand_t read_operand()
{
  operand_t o;
  int c;

  // Find operand
  while ((c = my_getchar()) != EOF && c != '\n' && c != '%' && c != '=' && !isalpha(c)) { }
  if (c == EOF || c == '\n') return (operand_t){ .ty = NONE, .n = 0 };
  if (c == '%' || c == '=') {
    o.ty = (c == '%' ? REGISTER : IMMEDIATE);
    uint32_t n = 0;
    while ((c = my_getchar()) >= '0' && c <= '9') {
      n = n * 10 + (c - '0');
    }
    o.n = n;
    my_ungetchar(c);
  } else if (isalpha(c)) {
    int dir = my_getchar();
    if (dir == 'b') {
      o.ty = LABEL_B;
    } else if (dir == 'f') {
      o.ty = LABEL_F;
    } else {
      printf("Invalid direction %c\n", dir);
      return (operand_t){ .ty = NONE, .n = 0 };
    }
    o.n = tolower(c) - 'a';
  }

  return o;
}

enum mnemonic_word {
  MN_B_AL = 0x00, MN_B_EQ, MN_B_NE, MN_B_GT, MN_B_LT, MN_B_GE, MN_B_LE,
  MN_B_SGT, MN_B_SLT, MN_B_SGE, MN_B_SLE,
  MN_B_COUNT,

  MN_MOV = 0x01, MN_ADD, MN_SUB, MN_MUL,
  MN_AND, MN_OR, MN_XOR, MN_BIC, MN_LSR, MN_ASR, MN_LSL,
  MN_LD, MN_ST, MN_LDC, MN_LDCR,
  MN_BR, MN_BR_END = MN_BR + MN_B_COUNT,
  MN_BA, MN_BA_END = MN_BA + MN_B_COUNT,
  MN_SC,
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
  uint32_t n_assembled = 0;
  int c;
  line_num = 1;

  uint32_t backward_labels[26];
  for (uint8_t i = 0; i < 26; i++) backward_labels[i] = (uint32_t)-1;
  uint32_t forward_patch[26][32], forward_patch_first_line[26];
  uint8_t forward_patch_n[26] = { 0 };

#define report_error_line(_line, _fmt, ...) do { \
  printf("line %u: " _fmt "\n", _line, ##__VA_ARGS__); \
  return n_assembled; \
} while (0)
#define report_error(_fmt, ...) report_error_line(line_num, _fmt, ##__VA_ARGS__)

#define emit(_opcode, _v1, _v2, _v3) \
  (rom[n_assembled++] = ((_opcode) << 24) | ((_v1) << 16) | ((_v2) << 8) | ((_v3) << 0))
#define emit_bh(_opcode, _v1, _v2) \
  (rom[n_assembled++] = ((_opcode) << 24) | ((_v1) << 16) | ((_v2) << 0))
#define emit_24(_opcode, _v) (rom[n_assembled++] = ((_opcode) << 24) | (_v))

  while (1) {
    while ((c = my_getchar()) != EOF && isspace(c)) { }
    if (c == EOF) break;

    uint32_t start_line_num = line_num;

    // Mnemonic
    uint8_t trie_pos = 0;
    uint8_t mnemonic_valid = 1; // Cleared when a non-existent child is encountered
    uint8_t mnemonic = 0;       // Word tag
    uint8_t label_id = 0, len = 0;
    uint8_t is_label = 0;
    do {
      c = tolower(c);
      trie_pos = mnemonic_trie[trie_pos][c - 'a' + 1];
      if (++len == 1) label_id = c - 'a';
      if (trie_pos == 0) {
        mnemonic_valid = 0;
      }
    } while ((c = my_getchar()) != EOF && isalpha(c));
    if (c == ':') {
      if (len != 1) {
        report_error("Invalid label");
      } else {
        is_label = 1;
      }
    } else {
      if (!mnemonic_valid || (mnemonic = mnemonic_trie[trie_pos][0]) == 0) {
        report_error("Invalid mnemonic");
      }
    }

    // Update labels
    if (is_label) {
      backward_labels[label_id] = n_assembled;
      for (uint8_t i = 0; i < forward_patch_n[label_id]; i++) {
        uint32_t addr = forward_patch[label_id][i];
        if (n_assembled - (addr + 1) > 0xff)
          report_error("Label too far away");
        rom[addr] |= (n_assembled - (addr + 1));
      }
      forward_patch_n[label_id] = 0;
      continue;
    }

    // Operands
    uint8_t n_operands = 0;
    operand_t operands[3];
    while (1) {
      operand_t o = read_operand();
      if (o.ty == NONE) break;
      if (n_operands >= 3) {
        report_error("Extraneous operand");
      } else {
        operands[n_operands++] = o;
      }
    }

    // Emit code
    if (mnemonic_valid && mnemonic > 0) {
      if (mnemonic >= MN_MOV && mnemonic <= MN_LSL) {
        if (n_operands == 3 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER &&
            operands[2].ty == REGISTER) {
          emit(0x10 + (mnemonic - MN_MOV), operands[0].n, operands[1].n, operands[2].n);
        } else if (n_operands == 3 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER &&
            operands[2].ty == IMMEDIATE) {
          emit(0x20 + (mnemonic - MN_MOV), operands[0].n, operands[1].n, operands[2].n);
        } else if (n_operands == 2 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER) {
          emit(0x10 + (mnemonic - MN_MOV), operands[0].n, operands[0].n, operands[1].n);
        } else if (n_operands == 2 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == IMMEDIATE) {
          emit_bh(0x30 + (mnemonic - MN_MOV), operands[0].n, operands[1].n);
        } else {
          report_error("Invalid operands");
        }
      } else if (mnemonic >= MN_BR && mnemonic < MN_BR_END) {
        if (n_operands == 3 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER &&
            operands[2].ty == REGISTER) {
          emit(0x90 + (mnemonic - MN_BR), operands[0].n, operands[1].n, operands[2].n);
        } else if (n_operands == 3 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER &&
            (operands[2].ty == LABEL_B || operands[2].ty == LABEL_F)) {
          uint8_t offset = 0;
          if (operands[2].ty == LABEL_B) {
            uint32_t last = backward_labels[operands[2].n];
            if (last == (uint32_t)-1)
              report_error("Label not found");
            if (last + 0x80 < n_assembled + 1)
              report_error("Label too far away");
            offset = (last - (n_assembled + 1)) & 0xff;
          } else {  // LABEL_F
            if (forward_patch_n[operands[2].n] >= sizeof forward_patch[0])
              report_error("Too many unresolved forward-referencing labels");
            forward_patch[operands[2].n][forward_patch_n[operands[2].n]++] = n_assembled;
            if (forward_patch_n[operands[2].n] == 1)
              forward_patch_first_line[operands[2].n] = start_line_num;
          }
          emit(0x80 + (mnemonic - MN_BR), operands[0].n, operands[1].n, offset);
        } else {
          report_error("Invalid operands");
        }
      } else if (mnemonic == MN_SC) {
        if (n_operands == 1 &&
            operands[0].ty == IMMEDIATE) {
          emit_24(0x00, operands[0].n);
        } else {
          report_error("Invalid operands");
        }
      } else {
        report_error("Unsupported instruction");
      }
    }
  }

  for (uint8_t i = 0; i < 26; i++)
    if (forward_patch_n[i] != 0)
      report_error_line(forward_patch_first_line[i], "Label not found");

  return n_assembled;
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
