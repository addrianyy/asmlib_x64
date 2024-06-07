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

  // region datatype_declarations

#define DECLARE_DATATYPE_HELPER(name, type)    \
  void name(type value) { push_value(value); } \
  void name(std::span<const type> values) { push_values(values); }

#define DECLARE_DATATYPE(name, type1, type2) \
  DECLARE_DATATYPE_HELPER(name, type1)       \
  DECLARE_DATATYPE_HELPER(name, type2)

  DECLARE_DATATYPE(db, int8_t, uint8_t)
  DECLARE_DATATYPE(dw, int16_t, uint16_t)
  DECLARE_DATATYPE(dd, int32_t, uint32_t)
  DECLARE_DATATYPE(dq, int64_t, uint64_t)

#undef DECLARE_DATATYPE_HELPER
#undef DECLARE_DATATYPE

  // endregion

  // region instruction_declarations

#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#define ENCODING_NAME(name) _asm_##name##_encoding

#define DECLARE_INSTRUCTION_0(name)                                      \
  void name() {                                                          \
    extern const encoding::FullInstructionEncoding* ENCODING_NAME(name); \
    encode_0(STRINGIFY(name), *ENCODING_NAME(name));                     \
  }
#define DECLARE_INSTRUCTION_1(name)                                      \
  void name(const Operand& op0) {                                        \
    extern const encoding::FullInstructionEncoding* ENCODING_NAME(name); \
    encode_1(STRINGIFY(name), *ENCODING_NAME(name), op0);                \
  }
#define DECLARE_INSTRUCTION_2(name)                                      \
  void name(const Operand& op0, const Operand& op1) {                    \
    extern const encoding::FullInstructionEncoding* ENCODING_NAME(name); \
    encode_2(STRINGIFY(name), *ENCODING_NAME(name), op0, op1);           \
  }
#define DECLARE_CONDITIONAL(jcc_name, cmov_name, setcc_name) \
  DECLARE_INSTRUCTION_1(jcc_name)                            \
  DECLARE_INSTRUCTION_2(cmov_name)                           \
  DECLARE_INSTRUCTION_1(setcc_name)

  DECLARE_INSTRUCTION_2(add)
  DECLARE_INSTRUCTION_2(sub)
  DECLARE_INSTRUCTION_2(xor_)
  DECLARE_INSTRUCTION_2(and_)
  DECLARE_INSTRUCTION_2(or_)
  DECLARE_INSTRUCTION_2(cmp)

  DECLARE_INSTRUCTION_2(shl)
  DECLARE_INSTRUCTION_2(shr)
  DECLARE_INSTRUCTION_2(sar)
  DECLARE_INSTRUCTION_2(rol)
  DECLARE_INSTRUCTION_2(ror)

  DECLARE_CONDITIONAL(ja, cmova, seta)
  DECLARE_CONDITIONAL(jae, cmovae, setae)
  DECLARE_CONDITIONAL(jb, cmovb, setb)
  DECLARE_CONDITIONAL(jbe, cmovbe, setbe)
  DECLARE_CONDITIONAL(jc, cmovc, setc)
  DECLARE_CONDITIONAL(je, cmove, sete)
  DECLARE_CONDITIONAL(jz, cmovz, setz)
  DECLARE_CONDITIONAL(jg, cmovg, setg)
  DECLARE_CONDITIONAL(jge, cmovge, setge)
  DECLARE_CONDITIONAL(jl, cmovl, setl)
  DECLARE_CONDITIONAL(jle, cmovle, setle)
  DECLARE_CONDITIONAL(jna, cmovna, setna)
  DECLARE_CONDITIONAL(jnae, cmovnae, setnae)
  DECLARE_CONDITIONAL(jnb, cmovnb, setnb)
  DECLARE_CONDITIONAL(jnbe, cmovnbe, setnbe)
  DECLARE_CONDITIONAL(jnc, cmovnc, setnc)
  DECLARE_CONDITIONAL(jne, cmovne, setne)
  DECLARE_CONDITIONAL(jng, cmovng, setng)
  DECLARE_CONDITIONAL(jnge, cmovnge, setnge)
  DECLARE_CONDITIONAL(jnl, cmovnl, setnl)
  DECLARE_CONDITIONAL(jnle, cmovnle, setnle)
  DECLARE_CONDITIONAL(jno, cmovno, setno)
  DECLARE_CONDITIONAL(jnp, cmovnp, setnp)
  DECLARE_CONDITIONAL(jns, cmovns, setns)
  DECLARE_CONDITIONAL(jnz, cmovnz, setnz)
  DECLARE_CONDITIONAL(jo, cmovo, seto)
  DECLARE_CONDITIONAL(jp, cmovp, setp)
  DECLARE_CONDITIONAL(jpe, cmovpe, setpe)
  DECLARE_CONDITIONAL(jpo, cmovpo, setpo)
  DECLARE_CONDITIONAL(js, cmovs, sets)

  DECLARE_INSTRUCTION_2(bt)
  DECLARE_INSTRUCTION_2(btc)
  DECLARE_INSTRUCTION_2(btr)
  DECLARE_INSTRUCTION_2(bts)

  DECLARE_INSTRUCTION_1(inc)
  DECLARE_INSTRUCTION_1(dec)
  DECLARE_INSTRUCTION_1(not_)
  DECLARE_INSTRUCTION_1(neg)
  DECLARE_INSTRUCTION_1(mul)
  DECLARE_INSTRUCTION_1(imul)
  DECLARE_INSTRUCTION_2(imul)
  DECLARE_INSTRUCTION_1(div)
  DECLARE_INSTRUCTION_1(idiv)

  DECLARE_INSTRUCTION_0(cqo)
  DECLARE_INSTRUCTION_0(int3)
  DECLARE_INSTRUCTION_0(rdtsc)
  DECLARE_INSTRUCTION_0(nop)

  DECLARE_INSTRUCTION_2(movzxb)
  DECLARE_INSTRUCTION_2(movzxw)
  DECLARE_INSTRUCTION_2(movsxb)
  DECLARE_INSTRUCTION_2(movsxw)
  DECLARE_INSTRUCTION_2(movsxd)

  DECLARE_INSTRUCTION_2(mov)
  DECLARE_INSTRUCTION_2(test)

  DECLARE_INSTRUCTION_1(push)
  DECLARE_INSTRUCTION_1(pop)
  DECLARE_INSTRUCTION_1(jmp)
  DECLARE_INSTRUCTION_1(call)

  DECLARE_INSTRUCTION_0(ret)
  DECLARE_INSTRUCTION_1(ret)

  DECLARE_INSTRUCTION_2(lea)

#undef DECLARE_CONDITIONAL
#undef DECLARE_INSTRUCTION_2
#undef DECLARE_INSTRUCTION_1
#undef DECLARE_INSTRUCTION_0

#undef ENCODING_NAME
#undef STRINGIFY
#undef STRINGIFY_HELPER

  // endregion
};

}  // namespace asmlib::x64
