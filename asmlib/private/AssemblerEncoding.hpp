#pragma once
#include <optional>

namespace asmlib {

enum class RexwMode {
  ExplicitRequired,
  Implicit,
  Usable,
  Unneeded,
};

enum class Prefix66Mode {
  Unusable,
  Usable,
  Unneeded,
};

struct SmallArray {
  uint8_t bytes[15]{};
  size_t size = 0;

  SmallArray(std::initializer_list<uint8_t> list) {
    for (auto element : list) {
      bytes[size++] = element;
    }
  }
};

struct Opcode {
  SmallArray op;
};

struct OpcodeDigit {
  SmallArray op;
  uint8_t digit;
};

struct OpcodeRegadd {
  SmallArray op;
};

struct InstructionEncoding {
  RexwMode rexw = RexwMode::Usable;
  Prefix66Mode p66 = Prefix66Mode::Usable;

  /// Instruction may require empty REX prefix to encode proper 8-bit register operand.
  bool fix_8bit = false;

  /// r64, r/m64
  std::optional<Opcode> regreg;

  /// r/m64, r64
  std::optional<Opcode> regreg_inv;

  std::optional<OpcodeDigit> regimm32;
  std::optional<OpcodeDigit> memimm32;
  std::optional<OpcodeDigit> regimm8;
  std::optional<OpcodeDigit> memimm8;
  std::optional<OpcodeDigit> reguimm8;
  std::optional<OpcodeDigit> memuimm8;
  std::optional<Opcode> regmem;
  std::optional<Opcode> memreg;
  std::optional<OpcodeDigit> regcl;
  std::optional<OpcodeDigit> memcl;
  std::optional<OpcodeDigit> reg;
  std::optional<OpcodeDigit> mem;
  std::optional<Opcode> rel32;
  std::optional<Opcode> imm32;
  std::optional<Opcode> uimm16;
  std::optional<OpcodeRegadd> regimm64;
  std::optional<Opcode> standalone;
};

struct FullInstructionEncoding {
  InstructionEncoding encoding_normal;
  std::optional<InstructionEncoding> encoding_8bit;

  explicit FullInstructionEncoding(InstructionEncoding normal,
                                   std::optional<InstructionEncoding> _8bit = std::nullopt)
      : encoding_normal(normal) {
    if (_8bit.has_value()) {
      _8bit->rexw = RexwMode::Unneeded;
      _8bit->p66 = Prefix66Mode::Unusable;
      _8bit->fix_8bit = true;
    }

    encoding_8bit = _8bit;
  }
};

} // namespace asmlib