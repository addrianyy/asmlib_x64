#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <vector>

#include "Operand.hpp"

namespace asmlib::x64 {

namespace encoding {
struct Array;
struct FullInstructionEncoding;
struct InstructionEncoding;
struct Opcode;
struct OpcodeDigit;
struct OpcodeRegadd;
}  // namespace encoding

namespace detail {
class EncodingGuard;
}

enum class OperandSize {
  Bits8 = 8,
  Bits16 = 16,
  Bits32 = 32,
  Bits64 = 64,
};

class Assembler {
  friend class detail::EncodingGuard;

  struct Fixup {
    Label label;
    size_t rel32_offset{};
    size_t end_offset{};
  };

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> labels;
  std::vector<Fixup> fixups;

  OperandSize operand_size = OperandSize::Bits64;

  bool force_rex = false;
  bool unfinished_fixup = false;

  void push_rex(bool w, bool r, bool x, bool b);
  void push_modrm(uint8_t mod, uint8_t reg, uint8_t rm);
  void push_sib(uint8_t base, uint8_t index, uint8_t scale);
  void push_imm(Imm imm, size_t size);
  void push_bytes(const encoding::Array& array);

  template <typename T>
  void push_value(T value) {
    const auto casted = std::bit_cast<std::array<uint8_t, sizeof(T)>>(value);

    bytes.reserve(sizeof(T));
    for (const auto b : casted) {
      bytes.push_back(b);
    }
  }
  template <typename T>
  void push_values(std::span<const T> values) {
    const auto previous_size = bytes.size();
    const auto data_size = values.size() * sizeof(T);

    bytes.resize(bytes.size() + data_size);
    std::memcpy(bytes.data() + previous_size, values.data(), data_size);
  }

  void require_64bit();
  void override_operand_size(const encoding::InstructionEncoding& encoding);
  bool get_rexw(const encoding::InstructionEncoding& encoding);

  void encode_memory_operand(uint8_t regop,
                             bool rex_r,
                             bool rex_w,
                             const encoding::Array& opcode,
                             Memory mem);

  void encode_regreg(Register reg1,
                     Register reg2,
                     const encoding::Opcode& op,
                     const encoding::InstructionEncoding& encoding);
  void encode_regimm(Register reg,
                     Imm imm,
                     size_t size,
                     const encoding::OpcodeDigit& op,
                     const encoding::InstructionEncoding& encoding);
  void encode_memreg_regmem(Register reg,
                            Memory mem,
                            const encoding::Opcode& op,
                            const encoding::InstructionEncoding& encoding);
  void encode_memimm(Memory mem,
                     Imm imm,
                     size_t size,
                     const encoding::OpcodeDigit& op,
                     const encoding::InstructionEncoding& encoding);
  void encode_mem(Memory mem,
                  const encoding::OpcodeDigit& op,
                  const encoding::InstructionEncoding& encoding);
  void encode_reg(Register reg,
                  const encoding::OpcodeDigit& op,
                  const encoding::InstructionEncoding& encoding);
  void encode_imm(Imm imm,
                  size_t size,
                  const encoding::Opcode& op,
                  const encoding::InstructionEncoding& encoding);
  void encode_standalone(const encoding::Opcode& op, const encoding::InstructionEncoding& encoding);
  void encode_rel32(int32_t rel,
                    Label label,
                    const encoding::Opcode& op,
                    const encoding::InstructionEncoding& encoding);
  void encode_regimm64(Register reg,
                       Imm imm,
                       const encoding::OpcodeRegadd& op,
                       const encoding::InstructionEncoding& encoding);

  const encoding::InstructionEncoding& preprocess_instruction(
    std::string_view name,
    const encoding::FullInstructionEncoding& encoding,
    std::span<Operand const* const> operands);
  void finalize_encoding();

  void encode_0(std::string_view name, const encoding::FullInstructionEncoding& encoding);
  void encode_1(std::string_view name,
                const encoding::FullInstructionEncoding& encoding,
                const Operand& op0);
  void encode_2(std::string_view name,
                const encoding::FullInstructionEncoding& encoding,
                const Operand& op0,
                const Operand& op1);

  void apply_fixups();

 public:
  Assembler() = default;

  void set_operand_size(OperandSize size) { operand_size = size; }

  template <typename Fn>
  void with_operand_size(OperandSize size, Fn fn) {
    const auto previous = operand_size;

    operand_size = size;
    fn();
    operand_size = previous;
  }

  uintptr_t current_offset() const { return bytes.size(); }

  Label allocate_label();
  void insert_label(Label label);
  Label insert_label();

  bool is_label_inserted(Label label) const;

  std::span<const uint8_t> assembled_instructions();

  void clear();

#define X64_ASM_DECLARE_DATATYPE_HELPER(name, type) \
  void name(type value) { push_value(value); }      \
  void name(std::span<const type> values) { push_values(values); }
#define X64_ASM_DECLARE_DATATYPE(name, type1, type2) \
  X64_ASM_DECLARE_DATATYPE_HELPER(name, type1)       \
  X64_ASM_DECLARE_DATATYPE_HELPER(name, type2)

  X64_ASM_DECLARE_DATATYPE(db, int8_t, uint8_t)
  X64_ASM_DECLARE_DATATYPE(dw, int16_t, uint16_t)
  X64_ASM_DECLARE_DATATYPE(dd, int32_t, uint32_t)
  X64_ASM_DECLARE_DATATYPE(dq, int64_t, uint64_t)

#undef X64_ASM_DECLARE_DATATYPE_HELPER
#undef X64_ASM_DECLARE_DATATYPE

#define X64_ASM_INSTRUCTION_0(name) void name();
#define X64_ASM_INSTRUCTION_1(name) void name(const Operand& op0);
#define X64_ASM_INSTRUCTION_2(name) void name(const Operand& op0, const Operand& op1);

#include "Instructions.inc"

#undef X64_ASM_INSTRUCTION_2
#undef X64_ASM_INSTRUCTION_1
#undef X64_ASM_INSTRUCTION_0
};

}  // namespace asmlib::x64
