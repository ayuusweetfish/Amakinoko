#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

  MN_DATA,
};
static uint8_t mnemonic_trie[80][27];

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
    {"data", MN_DATA},
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
        mnemonic_trie[p][0] = mnemonics[i].id + conditions[j].id;
      }
      mnemonic_trie[p][0] = mnemonics[i].id + MN_B_AL;
    } else {
      mnemonic_trie[p][0] = mnemonics[i].id;
    }
  }
}

static int hex_digit(int c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static uint32_t line_num, col_num, saved_col_num;

static inline int my_getchar()
{
  int c = getchar();
  if (c == '\n') { line_num++; saved_col_num = col_num; col_num = 0; }
  else col_num++;
  return c;
}
static inline void my_ungetchar(int c)
{
  ungetc(c, stdin);
  if (c == '\n') { line_num--; col_num = saved_col_num; }
  else col_num--;
}

struct file_pos_t {
  uint32_t line, col;
};

uint32_t assemble(
  uint32_t *restrict rom, uint32_t limit,
  struct file_pos_t *o_err_pos, char *o_err_msg, uint32_t err_msg_len_limit)
{
  uint32_t n_assembled = 0;
  int c;
  line_num = 1;
  col_num = 0;

  uint32_t backward_labels[26];
  for (uint8_t i = 0; i < 26; i++) backward_labels[i] = (uint32_t)-1;
  struct file_pos_t backward_labels_pos[26];  // Positions of labels

  struct patch_entry_t { uint32_t addr; uint8_t is_rel; } forward_patch[26][32];
  struct file_pos_t forward_patch_first_pos[26];  // Positions of each first patched operand
  uint8_t forward_patch_n[26] = { 0 };

#define report_error_pos(_pos, _fmt, ...) do { \
  snprintf(o_err_msg, err_msg_len_limit, _fmt, ##__VA_ARGS__); \
  *o_err_pos = (_pos); \
  return n_assembled; \
} while (0)
#define report_error(_line, _fmt, ...) \
  report_error_pos(((struct file_pos_t){ .line = (_line), .col = 0 }), _fmt, ##__VA_ARGS__)
#define report_error_here(_fmt, ...) \
  report_error_pos(((struct file_pos_t){ .line = line_num, .col = col_num }), _fmt, ##__VA_ARGS__)

#define emit_32(_v) do { \
  if (n_assembled >= limit) report_error(line_num, "Too many instructions"); \
  rom[n_assembled++] = (_v); \
} while (0)
#define emit_24(_opcode, _v1)        emit_32(((uint32_t)(_opcode) << 24) | ((_v1) <<  0))
#define emit(_opcode, _v1, _v2, _v3) emit_32(((uint32_t)(_opcode) << 24) | ((_v1) << 16) | ((_v2) << 8) | ((_v3) << 0))
#define emit_bh(_opcode, _v1, _v2)   emit_32(((uint32_t)(_opcode) << 24) | ((_v1) << 16) | ((_v2) << 0))

#define process_label_offset(_op, _op_pos, _is_rel, _offset) do { \
  operand_t op = (_op); \
  struct file_pos_t op_pos = (_op_pos); \
  bool is_rel = (_is_rel); \
  if (op.ty == LABEL_B) { \
    uint32_t last = backward_labels[op.n]; \
    if (last == (uint32_t)-1) \
      report_error_pos(op_pos, "Label %c not found", 'A' + op.n); \
    if (is_rel) { \
      if (last + 0x80 < n_assembled + 1) \
        report_error_pos(op_pos, "Relative label %c (%u:%u) too far away from backward-reference: distance is %u", \
          'A' + op.n, \
          backward_labels_pos[op.n].line, backward_labels_pos[op.n].col, \
          (n_assembled + 1) - last); \
      *(uint8_t *)(_offset) = (last - (n_assembled + 1)) & 0xff; \
    } else { \
      if (last > 0xffff) \
        report_error_pos(op_pos, \
          "Absolute label %c (%u:%u) out of 16-bit range: actual address is %08x", \
          'A' + op.n, \
          backward_labels_pos[op.n].line, backward_labels_pos[op.n].col, \
          last); \
      *(uint16_t *)(_offset) = last; \
    } \
  } else {  /* LABEL_F */ \
    if (forward_patch_n[op.n] >= sizeof forward_patch / sizeof forward_patch[0]) \
      report_error_pos(op_pos, \
        "Too many unresolved forward-referencing labels %cf", 'A' + op.n); \
    forward_patch[op.n][forward_patch_n[op.n]++] = \
      (struct patch_entry_t){ .addr = n_assembled, .is_rel = is_rel }; \
    if (forward_patch_n[op.n] == 1) \
      forward_patch_first_pos[op.n] = op_pos; \
  } \
} while (0)

  while (1) {
    while ((c = my_getchar()) != EOF && isspace(c)) { }
    if (c == EOF) break;
    if (c == '#') { // Comment
      while ((c = my_getchar()) != EOF && c != '\n') { }
      continue;
    }

    uint32_t instr_ln = line_num;
    struct file_pos_t instr_start_pos = { .line = line_num, .col = col_num };

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
        report_error(instr_ln, "Invalid label");
      } else {
        is_label = 1;
      }
    } else {
      if (!mnemonic_valid || (mnemonic = mnemonic_trie[trie_pos][0]) == 0) {
        report_error(instr_ln, "Invalid mnemonic");
      }
    }

    // Update labels
    if (is_label) {
      backward_labels[label_id] = n_assembled;
      backward_labels_pos[label_id] = instr_start_pos;
      for (uint8_t i = 0; i < forward_patch_n[label_id]; i++) {
        uint32_t addr = forward_patch[label_id][i].addr;
        if (forward_patch[label_id][i].is_rel) {
          if (n_assembled - (addr + 1) > 0xff)
            report_error_pos(forward_patch_first_pos[label_id],
              "Relative label %c (%u:%u) too far away from forward-reference: distance is %u",
              'A' + label_id,
              instr_start_pos.line, instr_start_pos.col,
              n_assembled - (addr + 1));
          rom[addr] |= (n_assembled - (addr + 1));
        } else {
          if (n_assembled > 0xffff)
            report_error_pos(forward_patch_first_pos[label_id],
              "Absolute label %c (%u:%u) out of 16-bit range: actual address is %08x",
              'A' + label_id,
              instr_start_pos.line, instr_start_pos.col,
              n_assembled);
          rom[addr] |= n_assembled;
        }
      }
      forward_patch_n[label_id] = 0;
      continue;
    }

    // Directives
    if (mnemonic == MN_DATA) {
      uint32_t value = 0;
      uint8_t n_bits = 0;
      while (1) {
        while (isspace(c)) {
          if (c == '\n') goto line_fin;
          c = my_getchar();
        }
        int n = hex_digit(c);
        if (n == -1) report_error_here("Expected hexadecimal number");
        value = (value << 4) | n;
        if (++n_bits == 8) {
          emit_32(value);
          value = 0;
          n_bits = 0;
        }
        c = my_getchar();
      }
    line_fin:
      if (n_bits != 8) emit_32(value);
      continue;
    }

    // Operands
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

    uint8_t n_operands = 0;
    operand_t operands[3];
    struct file_pos_t operands_pos[3];
    while (1) {
      operand_t o;
      int c;

      // Find operand
      while ((c = my_getchar()) != EOF && c != '\n' && (isspace(c) || c == ',')) { }
      if (c == EOF || c == '\n') break;
      if (c == '#') { // Comment
        while ((c = my_getchar()) != EOF && c != '\n') { }
        break;
      }

      struct file_pos_t start_pos = { .line = line_num, .col = col_num };
      if (c == '%' || c == '=') {
        o.ty = (c == '%' ? REGISTER : IMMEDIATE);
        uint32_t n = 0;
        while ((c = my_getchar()) >= '0' && c <= '9') {
          n = n * 10 + (c - '0');
          uint32_t limit = (o.ty == REGISTER ? ((uint32_t)1 << 8) : ((uint32_t)1 << 24));
          if (n >= limit)
            report_error_pos(start_pos, "%s out of range: expected <= %u",
              o.ty == REGISTER ? "Register index" : "Immediate value",
              limit - 1);
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
          report_error_pos(start_pos, "Invalid direction %c\n", dir);
        }
        o.n = tolower(c) - 'a';
      }

      if (n_operands >= 3) {
        report_error_pos(start_pos, "Extraneous operand");
      } else {
        operands_pos[n_operands] = start_pos;
        operands[n_operands++] = o;
      }
    }

    // Emit code
    if (mnemonic_valid && mnemonic > 0) {
      #define ensure_imm_range(_op_id, _bits) do { \
        uint32_t imm = operands[(_op_id)].n; \
        if (imm >= ((uint32_t)1 << (_bits))) \
          report_error_pos(operands_pos[(_op_id)], \
            "Immediate value %u out of range: expected <= %u", \
            imm, ((uint32_t)1 << (_bits)) - 1); \
      } while (0)
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
          ensure_imm_range(2, 8);
          emit(0x20 + (mnemonic - MN_MOV), operands[0].n, operands[1].n, operands[2].n);
        } else if (n_operands == 2 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER) {
          emit(0x10 + (mnemonic - MN_MOV), operands[0].n, operands[0].n, operands[1].n);
        } else if (n_operands == 2 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == IMMEDIATE) {
          ensure_imm_range(1, 16);
          emit_bh(0x30 + (mnemonic - MN_MOV), operands[0].n, operands[1].n);
        } else if (n_operands == 2 &&
            operands[0].ty == REGISTER &&
            (operands[1].ty == LABEL_B || operands[1].ty == LABEL_F)) {
          uint16_t offset = 0;
          process_label_offset(operands[1], operands_pos[1], false, &offset);
          emit_bh(0x30 + (mnemonic - MN_MOV), operands[0].n, offset);
        } else {
          report_error(instr_ln, "Invalid operands");
        }
      } else if (mnemonic >= MN_LD && mnemonic <= MN_LDCR) {
        if (n_operands == 3 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER &&
            operands[2].ty == IMMEDIATE) {
          ensure_imm_range(2, 8);
          emit(0x70 + (mnemonic - MN_LD), operands[0].n, operands[1].n, operands[2].n);
        } else {
          report_error(instr_ln, "Invalid operands");
        }
      } else if (mnemonic >= MN_BR && mnemonic < MN_BR_END) {
        if (mnemonic == MN_BR + MN_B_AL && n_operands == 1 &&
            (operands[0].ty == LABEL_B || operands[0].ty == LABEL_F)) {
          // XXX: Temporary approach, refactor to unify label handling
          n_operands = 3;
          operands[2] = operands[0];
          operands_pos[2] = operands_pos[0];
          operands[0] = operands[1] = (operand_t){ .ty = REGISTER, .n = 0 };
        }
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
          process_label_offset(operands[2], operands_pos[2], true, &offset);
          emit(0x80 + (mnemonic - MN_BR), operands[0].n, operands[1].n, offset);
        } else {
          report_error(instr_ln, "Invalid operands");
        }
      } else if (mnemonic >= MN_BA && mnemonic < MN_BA_END) {
        if (mnemonic == MN_BA + MN_B_AL && n_operands == 1 && operands[0].ty == REGISTER) {
          emit(0xA0, operands[0].n, operands[0].n, operands[0].n);
        } else if (n_operands == 3 &&
            operands[0].ty == REGISTER &&
            operands[1].ty == REGISTER &&
            operands[2].ty == REGISTER) {
          emit(0xA0 + (mnemonic - MN_BA), operands[0].n, operands[1].n, operands[2].n);
        } else {
          report_error(instr_ln, "Invalid operands");
        }
      } else if (mnemonic == MN_SC) {
        if (n_operands == 1 &&
            operands[0].ty == IMMEDIATE) {
          ensure_imm_range(0, 24);
          emit_24(0x00, operands[0].n);
        } else {
          report_error(instr_ln, "Invalid operands");
        }
      } else {
        // Should not happen!
        report_error(instr_ln, "Unsupported instruction");
      }
    }
  }

  for (uint8_t i = 0; i < 26; i++)
    if (forward_patch_n[i] != 0)
      report_error_pos(forward_patch_first_pos[i], "Forward-reference label %c not resolved", 'A' + i);

  *o_err_pos = (struct file_pos_t){ 0 };
  return n_assembled;
}

int main()
{
  init_trie();

  uint32_t c[65536];
  struct file_pos_t err_pos;
  char err_msg[64];
  uint32_t len = assemble(c, sizeof c / sizeof c[0], &err_pos, err_msg, sizeof err_msg);

  for (uint32_t i = 0; i < len; i++)
    printf("%04x: %08x\n", i, c[i]);

  if (err_pos.line != 0)
    printf("(%u:%u): %s\n", err_pos.line, err_pos.col, err_msg);

  return 0;
}
