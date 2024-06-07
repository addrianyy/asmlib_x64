#include "InstructionEncoding.hpp"

using namespace asmlib::x64;
using namespace asmlib::x64::encoding;

#define EXPORT_ENCODING(name)                                    \
  namespace asmlib::x64 {                                        \
  const FullInstructionEncoding* _asm_##name##_encoding = &name; \
  }                                                              \
  const FullInstructionEncoding* _asm_##name##_encoding = &name;

#define EXPORT_ENCODING_CUSTOM_NAME(name, encoding)                  \
  namespace asmlib::x64 {                                            \
  const FullInstructionEncoding* _asm_##name##_encoding = &encoding; \
  }                                                                  \
  const FullInstructionEncoding* _asm_##name##_encoding = &encoding;

#define MAKE_ARRAY(...) \
  { __VA_ARGS__ }

#define SIMPLE_INSTRUCTION(name, reg_regmem, memreg_, regmem_imm32, digit) \
  static constexpr FullInstructionEncoding name(                           \
    InstructionEncoding{                                                   \
      .rexw = RexwMode::Usable,                                            \
      .p66 = Prefix66Mode::Usable,                                         \
      .regreg = Opcode{{reg_regmem}},                                      \
      .regimm32 = OpcodeDigit{{regmem_imm32}, digit},                      \
      .memimm32 = OpcodeDigit{{regmem_imm32}, digit},                      \
      .regimm8 = OpcodeDigit{{regmem_imm32 + 2}, digit},                   \
      .memimm8 = OpcodeDigit{{regmem_imm32 + 2}, digit},                   \
      .regmem = Opcode{{reg_regmem}},                                      \
      .memreg = Opcode{{memreg_}},                                         \
    },                                                                     \
    InstructionEncoding{                                                   \
      .regreg = Opcode{{reg_regmem - 1}},                                  \
      .regimm32 = OpcodeDigit{{regmem_imm32 - 1}, digit},                  \
      .memimm32 = OpcodeDigit{{regmem_imm32 - 1}, digit},                  \
      .regmem = Opcode{{reg_regmem - 1}},                                  \
      .memreg = Opcode{{memreg_ - 1}},                                     \
    });                                                                    \
  EXPORT_ENCODING(name)

SIMPLE_INSTRUCTION(add, 0x03, 0x01, 0x81, 0)
SIMPLE_INSTRUCTION(sub, 0x2b, 0x29, 0x81, 5)
SIMPLE_INSTRUCTION(xor_, 0x33, 0x31, 0x81, 6)
SIMPLE_INSTRUCTION(and_, 0x23, 0x21, 0x81, 4)
SIMPLE_INSTRUCTION(or_, 0x0b, 0x09, 0x81, 1)
SIMPLE_INSTRUCTION(cmp, 0x3b, 0x39, 0x81, 7)

#define SHIFT_INSTRUCTION(name, digit)           \
  static constexpr FullInstructionEncoding name( \
    InstructionEncoding{                         \
      .rexw = RexwMode::Usable,                  \
      .p66 = Prefix66Mode::Usable,               \
      .reguimm8 = OpcodeDigit{{0xc1}, digit},    \
      .memuimm8 = OpcodeDigit{{0xc1}, digit},    \
      .regcl = OpcodeDigit{{0xd3}, digit},       \
      .memcl = OpcodeDigit{{0xd3}, digit},       \
    },                                           \
    InstructionEncoding{                         \
      .reguimm8 = OpcodeDigit{{0xc0}, digit},    \
      .memuimm8 = OpcodeDigit{{0xc0}, digit},    \
      .regcl = OpcodeDigit{{0xd2}, digit},       \
      .memcl = OpcodeDigit{{0xd2}, digit},       \
    });                                          \
  EXPORT_ENCODING(name)

SHIFT_INSTRUCTION(shl, 4)
SHIFT_INSTRUCTION(shr, 5)
SHIFT_INSTRUCTION(sar, 7)
SHIFT_INSTRUCTION(rol, 0)
SHIFT_INSTRUCTION(ror, 1)

#define CONDITIONAL_INSTRUCTION(jcc_name, cmov_name, setcc_name, code)     \
  static constexpr FullInstructionEncoding jcc_name(InstructionEncoding{   \
    .rexw = RexwMode::Unneeded,                                            \
    .p66 = Prefix66Mode::Unneeded,                                         \
    .rel32 = Opcode{{0x0f, code}},                                         \
  });                                                                      \
  EXPORT_ENCODING(jcc_name)                                                \
  static constexpr FullInstructionEncoding cmov_name(InstructionEncoding{  \
    .rexw = RexwMode::Usable,                                              \
    .p66 = Prefix66Mode::Usable,                                           \
    .regreg = Opcode{{0x0f, code - 0x40}},                                 \
    .regmem = Opcode{{0x0f, code - 0x40}},                                 \
  });                                                                      \
  EXPORT_ENCODING(cmov_name)                                               \
  static constexpr FullInstructionEncoding setcc_name(InstructionEncoding{ \
    .rexw = RexwMode::Unneeded,                                            \
    .p66 = Prefix66Mode::Unneeded,                                         \
    .fix_8bit = true,                                                      \
    .reg = OpcodeDigit{{0x0f, code + 0x10}, 0},                            \
    .mem = OpcodeDigit{{0x0f, code + 0x10}, 0},                            \
  });                                                                      \
  EXPORT_ENCODING(setcc_name)

CONDITIONAL_INSTRUCTION(ja, cmova, seta, 0x87)
CONDITIONAL_INSTRUCTION(jae, cmovae, setae, 0x83)
CONDITIONAL_INSTRUCTION(jb, cmovb, setb, 0x82)
CONDITIONAL_INSTRUCTION(jbe, cmovbe, setbe, 0x86)
CONDITIONAL_INSTRUCTION(jc, cmovc, setc, 0x82)
CONDITIONAL_INSTRUCTION(je, cmove, sete, 0x84)
CONDITIONAL_INSTRUCTION(jz, cmovz, setz, 0x84)
CONDITIONAL_INSTRUCTION(jg, cmovg, setg, 0x8f)
CONDITIONAL_INSTRUCTION(jge, cmovge, setge, 0x8d)
CONDITIONAL_INSTRUCTION(jl, cmovl, setl, 0x8c)
CONDITIONAL_INSTRUCTION(jle, cmovle, setle, 0x8e)
CONDITIONAL_INSTRUCTION(jna, cmovna, setna, 0x86)
CONDITIONAL_INSTRUCTION(jnae, cmovnae, setnae, 0x82)
CONDITIONAL_INSTRUCTION(jnb, cmovnb, setnb, 0x83)
CONDITIONAL_INSTRUCTION(jnbe, cmovnbe, setnbe, 0x87)
CONDITIONAL_INSTRUCTION(jnc, cmovnc, setnc, 0x83)
CONDITIONAL_INSTRUCTION(jne, cmovne, setne, 0x85)
CONDITIONAL_INSTRUCTION(jng, cmovng, setng, 0x8e)
CONDITIONAL_INSTRUCTION(jnge, cmovnge, setnge, 0x8c)
CONDITIONAL_INSTRUCTION(jnl, cmovnl, setnl, 0x8d)
CONDITIONAL_INSTRUCTION(jnle, cmovnle, setnle, 0x8f)
CONDITIONAL_INSTRUCTION(jno, cmovno, setno, 0x81)
CONDITIONAL_INSTRUCTION(jnp, cmovnp, setnp, 0x8b)
CONDITIONAL_INSTRUCTION(jns, cmovns, setns, 0x89)
CONDITIONAL_INSTRUCTION(jnz, cmovnz, setnz, 0x85)
CONDITIONAL_INSTRUCTION(jo, cmovo, seto, 0x80)
CONDITIONAL_INSTRUCTION(jp, cmovp, setp, 0x8a)
CONDITIONAL_INSTRUCTION(jpe, cmovpe, setpe, 0x8a)
CONDITIONAL_INSTRUCTION(jpo, cmovpo, setpo, 0x8b)
CONDITIONAL_INSTRUCTION(js, cmovs, sets, 0x88)

#define BIT_INSTRUCTION(name, opcode, digit)                         \
  static constexpr FullInstructionEncoding name(InstructionEncoding{ \
    .rexw = RexwMode::Usable,                                        \
    .p66 = Prefix66Mode::Usable,                                     \
    .regreg_inv = Opcode{{0x0f, opcode}},                            \
    .reguimm8 = OpcodeDigit{{0x0f, 0xba}, digit},                    \
    .memuimm8 = OpcodeDigit{{0x0f, 0xba}, digit},                    \
    .memreg = Opcode{{0x0f, opcode}},                                \
  });                                                                \
  EXPORT_ENCODING(name)

BIT_INSTRUCTION(bt, 0xa3, 4)
BIT_INSTRUCTION(btc, 0xbb, 7)
BIT_INSTRUCTION(btr, 0xb3, 6)
BIT_INSTRUCTION(bts, 0xab, 5)

#define UNARY_ARITH_INSTRUCTION(name, opcode, digit) \
  static constexpr FullInstructionEncoding name(     \
    InstructionEncoding{                             \
      .rexw = RexwMode::Usable,                      \
      .p66 = Prefix66Mode::Usable,                   \
      .reg = OpcodeDigit{{opcode}, digit},           \
      .mem = OpcodeDigit{{opcode}, digit},           \
    },                                               \
    InstructionEncoding{                             \
      .reg = OpcodeDigit{{opcode - 1}, digit},       \
      .mem = OpcodeDigit{{opcode - 1}, digit},       \
    });                                              \
  EXPORT_ENCODING(name)

UNARY_ARITH_INSTRUCTION(inc, 0xff, 0)
UNARY_ARITH_INSTRUCTION(dec, 0xff, 1)
UNARY_ARITH_INSTRUCTION(not_, 0xf7, 2)
UNARY_ARITH_INSTRUCTION(neg, 0xf7, 3)
UNARY_ARITH_INSTRUCTION(mul, 0xf7, 4)
UNARY_ARITH_INSTRUCTION(idiv, 0xf7, 7)

// Workaround for error caused by the fact that `div` function is already defined.
static constexpr FullInstructionEncoding div_(
  InstructionEncoding{
    .rexw = RexwMode::Usable,
    .p66 = Prefix66Mode::Usable,
    .reg = OpcodeDigit{{0xf7}, 6},
    .mem = OpcodeDigit{{0xf7}, 6},
  },
  InstructionEncoding{
    .reg = OpcodeDigit{{0xf7 - 1}, 6},
    .mem = OpcodeDigit{{0xf7 - 1}, 6},
  });
EXPORT_ENCODING_CUSTOM_NAME(div, div_)

#define STANDALONE_INSTRUCTION(name, bytes)      \
  static constexpr FullInstructionEncoding name( \
    InstructionEncoding{                         \
      .rexw = RexwMode::Unneeded,                \
      .p66 = Prefix66Mode::Unneeded,             \
      .standalone = Opcode{bytes},               \
    },                                           \
    InstructionEncoding{                         \
      .standalone = Opcode{bytes},               \
    });                                          \
  EXPORT_ENCODING(name)

STANDALONE_INSTRUCTION(int3, {0xcc})
STANDALONE_INSTRUCTION(rdtsc, MAKE_ARRAY(0x0f, 0x31))
STANDALONE_INSTRUCTION(nop, {0x90})

#define EXT_INSTRUCTION(name, rexw_, p66_, bytes, fix_8bit_)         \
  static constexpr FullInstructionEncoding name(InstructionEncoding{ \
    .rexw = (rexw_),                                                 \
    .p66 = (p66_),                                                   \
    .fix_8bit = (fix_8bit_),                                         \
    .regreg = Opcode{bytes},                                         \
    .regmem = Opcode{bytes},                                         \
  });                                                                \
  EXPORT_ENCODING(name)

EXT_INSTRUCTION(movzxb, RexwMode::Usable, Prefix66Mode::Usable, MAKE_ARRAY(0x0f, 0xb6), true)
EXT_INSTRUCTION(movzxw, RexwMode::Usable, Prefix66Mode::Unusable, MAKE_ARRAY(0x0f, 0xb7), false)
EXT_INSTRUCTION(movsxb, RexwMode::Usable, Prefix66Mode::Usable, MAKE_ARRAY(0x0f, 0xbe), true)
EXT_INSTRUCTION(movsxw, RexwMode::Usable, Prefix66Mode::Unusable, MAKE_ARRAY(0x0f, 0xbf), false)
EXT_INSTRUCTION(movsxd, RexwMode::ExplicitRequired, Prefix66Mode::Unusable, {0x63}, false)

static constexpr FullInstructionEncoding cqo(InstructionEncoding{
  .rexw = RexwMode::Usable,
  .p66 = Prefix66Mode::Usable,
  .standalone = Opcode{{0x99}},
});
EXPORT_ENCODING(cqo)

static constexpr FullInstructionEncoding mov(
  InstructionEncoding{
    .rexw = RexwMode::Usable,
    .p66 = Prefix66Mode::Usable,
    .regreg = Opcode{{0x8b}},
    .regimm32 = OpcodeDigit{{0xc7}, 0},
    .memimm32 = OpcodeDigit{{0xc7}, 0},
    .regmem = Opcode{{0x8b}},
    .memreg = Opcode{{0x89}},
    .regimm64 = OpcodeRegadd{{0xb8}},
  },
  InstructionEncoding{
    .regreg = Opcode{{0x8a}},
    .regimm32 = OpcodeDigit{{0xc6}, 0},
    .memimm32 = OpcodeDigit{{0xc6}, 0},
    .regmem = Opcode{{0x8a}},
    .memreg = Opcode{{0x88}},
  });
EXPORT_ENCODING(mov)

static constexpr FullInstructionEncoding test(
  InstructionEncoding{
    .rexw = RexwMode::Usable,
    .p66 = Prefix66Mode::Usable,
    .regreg_inv = Opcode{{0x85}},
    .regimm32 = OpcodeDigit{{0xf7}, 0},
    .memimm32 = OpcodeDigit{{0xf7}, 0},
    .memreg = Opcode{{0x85}},
  },
  InstructionEncoding{
    .regreg_inv = Opcode{{0x84}},
    .regimm32 = OpcodeDigit{{0xf6}, 0},
    .memimm32 = OpcodeDigit{{0xf6}, 0},
    .memreg = Opcode{{0x84}},
  });
EXPORT_ENCODING(test)

static constexpr FullInstructionEncoding push(InstructionEncoding{
  .rexw = RexwMode::Implicit,
  .p66 = Prefix66Mode::Usable,
  .reg = OpcodeDigit{{0xff}, 6},
  .mem = OpcodeDigit{{0xff}, 6},
  .imm32 = Opcode{{0x68}},
});
EXPORT_ENCODING(push)

static constexpr FullInstructionEncoding pop(InstructionEncoding{
  .rexw = RexwMode::Implicit,
  .p66 = Prefix66Mode::Usable,
  .reg = OpcodeDigit{{0x8f}, 0},
  .mem = OpcodeDigit{{0x8f}, 0},
});
EXPORT_ENCODING(pop)

static constexpr FullInstructionEncoding jmp(InstructionEncoding{
  .rexw = RexwMode::Unneeded,
  .p66 = Prefix66Mode::Unneeded,
  .reg = OpcodeDigit{{0xff}, 4},
  .mem = OpcodeDigit{{0xff}, 4},
  .rel32 = Opcode{{0xe9}},
});
EXPORT_ENCODING(jmp)

static constexpr FullInstructionEncoding call(InstructionEncoding{
  .rexw = RexwMode::Unneeded,
  .p66 = Prefix66Mode::Unneeded,
  .reg = OpcodeDigit{{0xff}, 2},
  .mem = OpcodeDigit{{0xff}, 2},
  .rel32 = Opcode{{0xe8}},
});
EXPORT_ENCODING(call)

static constexpr FullInstructionEncoding ret(InstructionEncoding{
  .rexw = RexwMode::Unneeded,
  .p66 = Prefix66Mode::Unneeded,
  .uimm16 = Opcode{{0xc2}},
  .standalone = Opcode{{0xc3}},
});
EXPORT_ENCODING(ret)

static constexpr FullInstructionEncoding imul(
  InstructionEncoding{
    .rexw = RexwMode::Usable,
    .p66 = Prefix66Mode::Usable,
    .regreg = Opcode{{0x0f, 0xaf}},
    .regmem = Opcode{{0x0f, 0xaf}},
    .reg = OpcodeDigit{{0xf7}, 5},
    .mem = OpcodeDigit{{0xf7}, 5},
  },
  InstructionEncoding{
    .reg = OpcodeDigit{{0xf6}, 5},
    .mem = OpcodeDigit{{0xf6}, 5},
  });
EXPORT_ENCODING(imul)

static constexpr FullInstructionEncoding lea(InstructionEncoding{
  .rexw = RexwMode::Usable,
  .p66 = Prefix66Mode::Usable,
  .regmem = Opcode{{0x8d}},
});
EXPORT_ENCODING(lea)
