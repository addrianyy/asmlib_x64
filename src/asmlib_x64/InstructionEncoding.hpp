#pragma once
#include <cstdint>
#include <initializer_list>
#include <utility>

namespace asmlib::x64::encoding {

namespace detail {
void trigger_consteval_failure(char const* message);
}

template <typename T>
struct Optional {
  T value{};
  bool has_value{false};

  consteval Optional() = default;
  consteval Optional(T value) : value(std::move(value)), has_value(true) {}

  constexpr operator bool() const { return has_value; }
  constexpr const T* operator->() const { return &value; }
  constexpr const T& operator*() const { return value; }
};

struct Array {
  // Increase when needed.
  constexpr static size_t Capacity = 2;

  uint8_t bytes[Capacity]{};
  uint8_t size = 0;

  consteval Array(std::initializer_list<uint8_t> elements) {
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
  Array op;
};

struct OpcodeDigit {
  Array op;
  uint8_t digit;
};

struct OpcodeRegadd {
  Array op;
};

struct InstructionEncoding {
  RexwMode rexw = RexwMode::Usable;
  Prefix66Mode p66 = Prefix66Mode::Usable;

  /// Instruction may require empty REX prefix to encode proper 8-bit register operand.
  bool fix_8bit = false;

  /// r64, r/m64
  Optional<Opcode> regreg;

  /// r/m64, r64
  Optional<Opcode> regreg_inv;

  Optional<OpcodeDigit> regimm32;
  Optional<OpcodeDigit> memimm32;
  Optional<OpcodeDigit> regimm8;
  Optional<OpcodeDigit> memimm8;
  Optional<OpcodeDigit> reguimm8;
  Optional<OpcodeDigit> memuimm8;
  Optional<Opcode> regmem;
  Optional<Opcode> memreg;
  Optional<OpcodeDigit> regcl;
  Optional<OpcodeDigit> memcl;
  Optional<OpcodeDigit> reg;
  Optional<OpcodeDigit> mem;
  Optional<Opcode> rel32;
  Optional<Opcode> imm32;
  Optional<Opcode> uimm16;
  Optional<OpcodeRegadd> regimm64;
  Optional<Opcode> standalone;
};

struct FullInstructionEncoding {
  InstructionEncoding encoding_normal;
  Optional<InstructionEncoding> encoding_8bit;

  explicit consteval FullInstructionEncoding(InstructionEncoding normal,
                                             Optional<InstructionEncoding> _8bit = {})
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

}  // namespace asmlib::x64::encoding