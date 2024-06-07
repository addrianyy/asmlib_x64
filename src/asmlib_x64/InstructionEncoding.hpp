#pragma once
#include <cstdint>
#include <initializer_list>
#include <optional>

namespace asmlib::x64 {

template <typename T>
struct TrivialOptional {
  T value{};
  bool has_value{false};

  constexpr TrivialOptional() = default;
  constexpr TrivialOptional(T value) : value(value), has_value(true) {}

  constexpr operator bool() const { return has_value; }
  constexpr const T* operator->() const { return &value; }
  constexpr const T& operator*() const { return value; }
};

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

  constexpr SmallArray(std::initializer_list<uint8_t> list) {
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
  TrivialOptional<Opcode> regreg;

  /// r/m64, r64
  TrivialOptional<Opcode> regreg_inv;

  TrivialOptional<OpcodeDigit> regimm32;
  TrivialOptional<OpcodeDigit> memimm32;
  TrivialOptional<OpcodeDigit> regimm8;
  TrivialOptional<OpcodeDigit> memimm8;
  TrivialOptional<OpcodeDigit> reguimm8;
  TrivialOptional<OpcodeDigit> memuimm8;
  TrivialOptional<Opcode> regmem;
  TrivialOptional<Opcode> memreg;
  TrivialOptional<OpcodeDigit> regcl;
  TrivialOptional<OpcodeDigit> memcl;
  TrivialOptional<OpcodeDigit> reg;
  TrivialOptional<OpcodeDigit> mem;
  TrivialOptional<Opcode> rel32;
  TrivialOptional<Opcode> imm32;
  TrivialOptional<Opcode> uimm16;
  TrivialOptional<OpcodeRegadd> regimm64;
  TrivialOptional<Opcode> standalone;
};

struct FullInstructionEncoding {
  InstructionEncoding encoding_normal;
  TrivialOptional<InstructionEncoding> encoding_8bit;

  explicit constexpr FullInstructionEncoding(
    InstructionEncoding normal,
    TrivialOptional<InstructionEncoding> _8bit = TrivialOptional<InstructionEncoding>{})
      : encoding_normal(normal) {
    if (_8bit.has_value) {
      _8bit.value.rexw = RexwMode::Unneeded;
      _8bit.value.p66 = Prefix66Mode::Unusable;
      _8bit.value.fix_8bit = true;
    }

    encoding_8bit = _8bit;
  }
};

}  // namespace asmlib::x64