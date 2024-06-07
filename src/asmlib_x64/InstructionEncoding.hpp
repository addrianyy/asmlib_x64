#pragma once
#include <cstdint>
#include <initializer_list>
#include <utility>

namespace asmlib::x64 {

namespace detail {
void trigger_consteval_failure(char const* message);
}

template <typename T>
struct EncodingOptional {
  T value{};
  bool has_value{false};

  consteval EncodingOptional() = default;
  consteval EncodingOptional(T value) : value(std::move(value)), has_value(true) {}

  constexpr operator bool() const { return has_value; }
  constexpr const T* operator->() const { return &value; }
  constexpr const T& operator*() const { return value; }
};

struct EncodingArray {
  // Increase when needed.
  constexpr static size_t Capacity = 2;

  uint8_t bytes[Capacity]{};
  uint8_t size = 0;

  consteval EncodingArray(std::initializer_list<uint8_t> elements) {
    if (elements.size() > Capacity) {
      detail::trigger_consteval_failure("overflowed SmallArray");
    }

    for (const auto element : elements) {
      bytes[size++] = element;
    }
  }
};

enum class RexwMode : uint8_t {
  ExplicitRequired,
  Implicit,
  Usable,
  Unneeded,
};

enum class Prefix66Mode : uint8_t {
  Unusable,
  Usable,
  Unneeded,
};

struct Opcode {
  EncodingArray op;
};

struct OpcodeDigit {
  EncodingArray op;
  uint8_t digit;
};

struct OpcodeRegadd {
  EncodingArray op;
};

struct InstructionEncoding {
  RexwMode rexw = RexwMode::Usable;
  Prefix66Mode p66 = Prefix66Mode::Usable;

  /// Instruction may require empty REX prefix to encode proper 8-bit register operand.
  bool fix_8bit = false;

  /// r64, r/m64
  EncodingOptional<Opcode> regreg;

  /// r/m64, r64
  EncodingOptional<Opcode> regreg_inv;

  EncodingOptional<OpcodeDigit> regimm32;
  EncodingOptional<OpcodeDigit> memimm32;
  EncodingOptional<OpcodeDigit> regimm8;
  EncodingOptional<OpcodeDigit> memimm8;
  EncodingOptional<OpcodeDigit> reguimm8;
  EncodingOptional<OpcodeDigit> memuimm8;
  EncodingOptional<Opcode> regmem;
  EncodingOptional<Opcode> memreg;
  EncodingOptional<OpcodeDigit> regcl;
  EncodingOptional<OpcodeDigit> memcl;
  EncodingOptional<OpcodeDigit> reg;
  EncodingOptional<OpcodeDigit> mem;
  EncodingOptional<Opcode> rel32;
  EncodingOptional<Opcode> imm32;
  EncodingOptional<Opcode> uimm16;
  EncodingOptional<OpcodeRegadd> regimm64;
  EncodingOptional<Opcode> standalone;
};

struct FullInstructionEncoding {
  InstructionEncoding encoding_normal;
  EncodingOptional<InstructionEncoding> encoding_8bit;

  explicit consteval FullInstructionEncoding(InstructionEncoding normal,
                                             EncodingOptional<InstructionEncoding> _8bit = {})
      : encoding_normal(normal) {
    if (_8bit.has_value) {
      auto& v = _8bit.value;
      v.rexw = RexwMode::Unneeded;
      v.p66 = Prefix66Mode::Unusable;
      v.fix_8bit = true;
    }

    encoding_8bit = _8bit;
  }
};

}  // namespace asmlib::x64