#pragma once
#include "Operand.hpp"
#include <array>
#include <unordered_map>
#include <vector>

namespace asmlib {

enum class OperandSize {
  Bits8 = 8,
  Bits16 = 16,
  Bits32 = 32,
  Bits64 = 64,
};

class Assembler {
  friend class EncodingGuard;

  using LabelID = size_t;

  struct Fixup {
    std::string label;
    size_t rel32_offset;
    size_t end_offset;
  };

  std::vector<uint8_t> bytes;
  std::unordered_map<std::string, size_t> labels;
  std::vector<Fixup> fixups;

  OperandSize operand_size = OperandSize::Bits64;

  bool force_rex = false;
  bool pending_fixup_fill = false;

  template <typename T, typename U> static bool fits_within(U value) {
    using Limits = std::numeric_limits<T>;

    const int64_t v = int64_t(value);

    return v <= int64_t(Limits::max()) && v >= int64_t(Limits::min());
  }

  /// Check if value fits within imm32 adjusted for operand size.
  template <typename T> bool fits_within_imm32(T value) const {
    switch (operand_size) {
    case OperandSize::Bits8:
      return fits_within<int8_t>(value);

    case OperandSize::Bits16:
      return fits_within<int16_t>(value);

    default:
      return fits_within<int32_t>(value);
    }
  }

  /// Get imm32 size adjusted for operand size.
  size_t get_imm32_size() const {
    switch (operand_size) {
    case OperandSize::Bits8:
      return 1;
    case OperandSize::Bits16:
      return 2;
    default:
      return 4;
    }
  }

  LabelID get_id_for_label(std::string_view label);

  void push_rex(bool w, bool r, bool x, bool b);
  void push_modrm(uint8_t mod, uint8_t reg, uint8_t rm);
  void push_sib(uint8_t base, uint8_t index, uint8_t scale);
  void push_imm(Imm imm, size_t size);
  void push_bytes(const struct SmallArray& array);

  template <typename T> void push_value(const T& value) {
    const auto casted = std::bit_cast<std::array<uint8_t, sizeof(T)>>(value);

    bytes.reserve(sizeof(T));
    for (const auto b : casted) {
      bytes.push_back(b);
    }
  }

  void require_64bit();
  void override_operand_size(const struct InstructionEncoding& encoding);
  bool get_rexw(const struct InstructionEncoding& encoding);

  void encode_memory_operand(uint8_t regop, bool rex_r, bool rex_w, const struct SmallArray& opcode,
                             Memory mem);

  void encode_regreg(Reg reg1, Reg reg2, const struct Opcode& op,
                     const struct InstructionEncoding& encoding);
  void encode_regimm(Reg reg, Imm imm, size_t size, const struct OpcodeDigit& op,
                     const struct InstructionEncoding& encoding);
  void encode_memreg_regmem(Reg reg, Memory mem, const struct Opcode& op,
                            const struct InstructionEncoding& encoding);
  void encode_memimm(Memory mem, Imm imm, size_t size, const struct OpcodeDigit& op,
                     const struct InstructionEncoding& encoding);
  void encode_mem(Memory mem, const struct OpcodeDigit& op,
                  const struct InstructionEncoding& encoding);
  void encode_reg(Reg reg, const struct OpcodeDigit& op,
                  const struct InstructionEncoding& encoding);
  void encode_imm(Imm imm, size_t size, const struct Opcode& op,
                  const struct InstructionEncoding& encoding);
  void encode_standalone(const struct Opcode& op, const struct InstructionEncoding& encoding);
  void encode_rel32(int32_t rel, const std::optional<std::string_view>& label,
                    const struct Opcode& op, const struct InstructionEncoding& encoding);
  void encode_regimm64(Reg reg, Imm imm, const struct OpcodeRegadd& op,
                       const struct InstructionEncoding& encoding);

  const struct InstructionEncoding&
  instruction_preprocess(std::string_view name, const struct FullInstructionEncoding& encoding,
                         const Operand** operands, size_t operands_count);
  void on_encoding_end();

  void encode_0(std::string_view name, const struct FullInstructionEncoding& encoding);
  void encode_1(std::string_view name, const struct FullInstructionEncoding& encoding,
                const Operand& op0);
  void encode_2(std::string_view name, const struct FullInstructionEncoding& encoding,
                const Operand& op0, const Operand& op1);

  void apply_fixups();

public:
  explicit Assembler(OperandSize size = OperandSize::Bits64) : operand_size(size) {}

  std::vector<uint8_t> get_assembled_bytes();
  std::vector<uint8_t> take_assembled_bytes();

  void label(std::string label);
  void set_operand_size(OperandSize size) { operand_size = size; }

  template <typename Fn> void with_operand_size(OperandSize size, Fn fn) {
    const auto previous = operand_size;

    operand_size = size;
    fn();
    operand_size = previous;
  }

  // region instruction_declarations

#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#define ENCODING_NAME(name) _asm_##name##_encoding

#define DECLARE_INSTRUCTION_0(name)                                                                \
  void name() {                                                                                    \
    extern const FullInstructionEncoding* ENCODING_NAME(name);                                     \
    encode_0(STRINGIFY(name), *ENCODING_NAME(name));                                               \
  }
#define DECLARE_INSTRUCTION_1(name)                                                                \
  void name(const Operand& op0) {                                                                  \
    extern const FullInstructionEncoding* ENCODING_NAME(name);                                     \
    encode_1(STRINGIFY(name), *ENCODING_NAME(name), op0);                                          \
  }
#define DECLARE_INSTRUCTION_2(name)                                                                \
  void name(const Operand& op0, const Operand& op1) {                                              \
    extern const FullInstructionEncoding* ENCODING_NAME(name);                                     \
    encode_2(STRINGIFY(name), *ENCODING_NAME(name), op0, op1);                                     \
  }
#define DECLARE_CONDITIONAL(jcc_name, cmov_name, setcc_name)                                       \
  DECLARE_INSTRUCTION_1(jcc_name)                                                                  \
  DECLARE_INSTRUCTION_2(cmov_name)                                                                 \
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

} // namespace asmlib