#include "Assembler.hpp"
#include "InstructionEncoding.hpp"

#if __has_include(<base/Error.hpp> )
#include <base/Error.hpp>
#define X64_ASM_ASSERT verify
#define X64_ASM_HAS_FORMATTED_ASSERT
#else
#include <cstdio>
#include <cstdlib>
#define X64_ASM_ASSERT(value, message)                  \
  do {                                                  \
    const auto _assert_value = (value);                 \
    if (!_assert_value) {                               \
      std::printf("asmlib_x64 error: %s\n", (message)); \
      std::exit(1);                                     \
    }                                                   \
  } while (0)
#endif

using namespace asmlib::x64;

constexpr uint8_t MOD_DIRECT = 0b11;

using OpTy = Operand::Type;

static std::pair<bool, uint8_t> get_register_encoding(Register reg) {
  switch (reg) {
    case Register::Rax:
      return {false, 0b000};
    case Register::Rbx:
      return {false, 0b011};
    case Register::Rcx:
      return {false, 0b001};
    case Register::Rdx:
      return {false, 0b010};
    case Register::Rsp:
      return {false, 0b100};
    case Register::Rbp:
      return {false, 0b101};
    case Register::Rsi:
      return {false, 0b110};
    case Register::Rdi:
      return {false, 0b111};
    case Register::R8:
      return {true, 0b000};
    case Register::R9:
      return {true, 0b001};
    case Register::R10:
      return {true, 0b010};
    case Register::R11:
      return {true, 0b011};
    case Register::R12:
      return {true, 0b100};
    case Register::R13:
      return {true, 0b101};
    case Register::R14:
      return {true, 0b110};
    case Register::R15:
      return {true, 0b111};
    default:
      return {false, 0b000};
  }
}

static bool get_register_ext(Register reg) {
  return get_register_encoding(reg).first;
}
static uint8_t get_register_id(Register reg) {
  return get_register_encoding(reg).second;
}

namespace asmlib::x64 {

class EncodingGuard {
  Assembler* assembler;

 public:
  explicit EncodingGuard(Assembler* assembler) : assembler(assembler) {}
  ~EncodingGuard() { assembler->on_encoding_end(); }
};

Label* Assembler::LabelPool::allocate() {
  if (chunks.empty() || last_chunk_size == chunk_max_size) {
    chunks.push_back(std::make_unique<Chunk>());
    last_chunk_size = 0;
  }

  return &chunks.back()->labels[last_chunk_size++];
}

Label* Assembler::get_or_create_named_label(std::string name) {
  Label* label_;

  const auto it = named_labels.find(name);
  if (it == named_labels.end()) {
    label_ = create_label();
    named_labels.insert({std::move(name), label_});
  } else {
    label_ = it->second;
  }

  return label_;
}

Label* Assembler::get_label(const detail::LabelOperand& label_operand) {
  if (label_operand.label_) {
    return label_operand.label_;
  }

  return get_or_create_named_label(std::string(label_operand.name_));
}

void Assembler::push_rex(bool w, bool r, bool x, bool b) {
  // Usually REX without any attributes doesn't need to be emited because it doesn't
  // change anything. 8 bit instructions operands are exception. For example:
  // 0b110 encodes DH without REX and SIL with REX. In this case we are required to emit
  // REX.
  if (!force_rex && !w && !r && !x && !b) {
    return;
  }

  const uint8_t w_ = w;
  const uint8_t r_ = r;
  const uint8_t x_ = x;
  const uint8_t b_ = b;

  bytes.push_back(uint8_t(0b0100) << 4 | w_ << 3 | r_ << 2 | x_ << 1 | b_);
}

void Assembler::push_modrm(uint8_t mod, uint8_t reg, uint8_t rm) {
  X64_ASM_ASSERT(mod <= 0b11, "Mod in modrm is too big.");
  X64_ASM_ASSERT(reg <= 0b111, "Reg in modrm is too big.");
  X64_ASM_ASSERT(rm <= 0b111, "Rm in modrm is too big.");

  bytes.push_back(mod << 6 | reg << 3 | rm);
}

void Assembler::push_sib(uint8_t base, uint8_t index, uint8_t scale) {
  X64_ASM_ASSERT(base <= 0b111, "Base in sib is too big.");
  X64_ASM_ASSERT(index <= 0b111, "Index in sib is too big.");
  X64_ASM_ASSERT(scale <= 0b11, "Scale in sib is too big.");

  bytes.push_back(scale << 6 | index << 3 | base);
}

void Assembler::push_imm(Imm imm, size_t size) {
  static_assert(sizeof(imm) == 8, "Immediate is not 64 bit.");

  X64_ASM_ASSERT(size == 1 || size == 2 || size == 4 || size == 8, "Unexpected immediate size");

  const auto casted = std::bit_cast<std::array<uint8_t, 8>>(imm);

  bool all_0s = true;
  bool all_fs = true;
  for (size_t i = size; i < casted.size(); ++i) {
    if (casted[i] != 0x00) {
      all_0s = false;
    }
    if (casted[i] != 0xff) {
      all_fs = false;
    }
  }

  X64_ASM_ASSERT(all_0s || all_fs, "Immediate truncation would cause data loss");

  bytes.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    bytes.push_back(casted[i]);
  }
}

void Assembler::push_bytes(const SmallArray& array) {
  bytes.reserve(array.size);
  for (size_t i = 0; i < array.size; ++i) {
    bytes.push_back(array.bytes[i]);
  }
}

void Assembler::require_64bit() {
  X64_ASM_ASSERT(operand_size == OperandSize::Bits64,
                 "This operation must be done with 64 bit operand size.");
}

void Assembler::override_operand_size(const InstructionEncoding& encoding) {
  if (operand_size == OperandSize::Bits16) {
    switch (encoding.p66) {
      case Prefix66Mode::Unusable:
        X64_ASM_ASSERT(false, "This operation cannot be done with 16 bit operand size.");
        break;

      case Prefix66Mode::Usable:
        bytes.push_back(0x66);
        break;

      case Prefix66Mode::Unneeded:
      default:
        break;
    }
  }
}

bool Assembler::get_rexw(const InstructionEncoding& encoding) {
  switch (encoding.rexw) {
    case RexwMode::Implicit:
      // Instructions with implicit REX.W can possibly be encoded with 16 bit operand
      // size (but not 32).
      if (operand_size == OperandSize::Bits16 && encoding.p66 == Prefix66Mode::Usable) {
        return false;
      }

      require_64bit();
      return false;

    case RexwMode::ExplicitRequired:
      require_64bit();
      return true;

    case RexwMode::Usable:
      return operand_size == OperandSize::Bits64;

    case RexwMode::Unneeded:
    default:
      return false;
  }
}

void Assembler::encode_memory_operand(uint8_t regop,
                                      bool rex_r,
                                      bool rex_w,
                                      const SmallArray& opcode,
                                      Memory mem) {
  constexpr uint8_t RM_SIB = 0b100;
  constexpr uint8_t RM_DISP = 0b101;

  if (const auto& label = mem.get_label(); label) {
    // Use RIP relative addressing to refer to the label.

    push_rex(rex_w, rex_r, false, false);
    push_bytes(opcode);
    push_modrm(0b00, regop, 0b101);

    // We don't know instruction end offset yet. It will be filled after encoding.
    const auto rel32_offset = bytes.size();
    fixups.push_back(Fixup{get_label(*label), rel32_offset, 0});
    pending_fixup_fill = true;

    // empty rel32 - will be filled by apply_fixups
    push_value(int32_t(0));

    return;
  }

  auto base = mem.get_base();
  auto index = mem.get_index();
  auto disp = mem.get_displacement() == 0 ? std::nullopt : std::optional(mem.get_displacement());

  // Set if operand contains index register but not base one. It requires special handling.
  bool index_no_base = false;

  // Set if we need SIB byte to encode memory operand.
  bool require_sib = false;

  if (base) {
    const auto encoding = get_register_id(*base);

    // SIB is required to encode RSP or R12 as a base register.
    if (encoding == RM_SIB) {
      require_sib = true;
    }

    // Displacement is required to encode RBP or R13 as a base register.
    if (encoding == RM_DISP && !disp) {
      disp = 0;
    }
  } else {
    // There must be at least one component in memory operand.
    X64_ASM_ASSERT(index || disp, "Memory operand is empty.");

    if (index) {
      index_no_base = true;

      // 32 bit displacement is required to encode memory operand with index register but
      // without base one.
      if (!disp) {
        disp = 0;
      }

      // Base register RBP will be ignored if MODRM.MOD == 0.
      base = Register::Rbp;
    }
  }

  if (index) {
    // Special case: R12 can be used as an index register even though it has
    // the same 3-bit encoding as RSP (which cannot).
    X64_ASM_ASSERT(index->index != Register::Rsp, "RSP cannot be used as an index register.");

    require_sib = true;
  }

  using ByteLimits = std::numeric_limits<uint8_t>;

  // It is possible to use 1 byte displacement if it fits in 8 bit signed integer.
  const bool byte_disp =
    disp && *disp >= int32_t(ByteLimits::min()) && *disp <= int32_t(ByteLimits::max());

  // If registers are not present they don't need extension in REX by default.
  const bool rex_b = base && get_register_ext(*base);
  const bool rex_x = index && get_register_ext(index->index);

  push_rex(rex_w, rex_r, rex_x, rex_b);
  push_bytes(opcode);

  if (!require_sib) {
    X64_ASM_ASSERT(!index_no_base, "There cannot be index register if SIB is not required.");

    if (disp) {
      if (base) {
        const auto base_encoding = get_register_id(*base);

        if (byte_disp) {
          // Encode base register and 8 bit displacement (without SIB).
          push_modrm(0b01, regop, base_encoding);
          push_value(int8_t(*disp));
        } else {
          // Encode base register and 32 bit displacement (without SIB).
          push_modrm(0b10, regop, base_encoding);
          push_value(int32_t(*disp));
        }
      } else {
        // In 64 bit mode the only way to encode displacement-only operand is to use
        // SIB byte with these special values. Displacement must be 32 bit wide.
        // Encoding without SIB is actually RIP-relative addressing.
        push_modrm(0b00, regop, RM_SIB);
        push_sib(0b101, 0b100, 0);
        push_value(int32_t(*disp));
      }
    } else {
      // Simple case: we just need to encode base register.
      push_modrm(0b00, regop, get_register_id(*base));
    }
  } else {
    if (disp && !index_no_base) {
      // Pick the shortest possible displacement encoding.
      if (byte_disp) {
        push_modrm(0b01, regop, RM_SIB);
      } else {
        push_modrm(0b10, regop, RM_SIB);
      }
    } else {
      // No displacement is needed or index_no_base is true. In that case
      // base will be RBP and 32-bit displacement will be still required.
      push_modrm(0b00, regop, RM_SIB);
    }

    // If index_no_base is set then MODRM.MOD == 00 and base register == RBP
    // so base register will be ignored.

    // RSP encoding means no index (scale is ignored then).
    const Register index_reg = index ? index->index : Register::Rsp;

    uint8_t x86_scale = 0;
    if (index) {
      switch (index->scale) {
        case 1:
          x86_scale = 0;
          break;
        case 2:
          x86_scale = 1;
          break;
        case 4:
          x86_scale = 2;
          break;
        case 8:
          x86_scale = 3;
          break;
        default:
          X64_ASM_ASSERT(false, "Only scales 1, 2, 4 and 8 are supported.");
      }
    }

    push_sib(get_register_id(*base), get_register_id(index_reg), x86_scale);

    if (disp) {
      if (byte_disp && !index_no_base) {
        push_value(int8_t(*disp));
      } else {
        push_value(int32_t(*disp));
      }
    }
  }
}

void Assembler::encode_regreg(Register reg1,
                              Register reg2,
                              const Opcode& op,
                              const InstructionEncoding& encoding) {
  const auto [reg1_e, reg1_enc] = get_register_encoding(reg1);
  const auto [reg2_e, reg2_enc] = get_register_encoding(reg2);

  override_operand_size(encoding);
  push_rex(get_rexw(encoding), reg1_e, false, reg2_e);
  push_bytes(op.op);
  push_modrm(MOD_DIRECT, reg1_enc, reg2_enc);
}

void Assembler::encode_regimm(Register reg,
                              Imm imm,
                              size_t size,
                              const OpcodeDigit& op,
                              const InstructionEncoding& encoding) {
  const auto [reg_e, reg_enc] = get_register_encoding(reg);

  override_operand_size(encoding);
  push_rex(get_rexw(encoding), false, false, reg_e);
  push_bytes(op.op);
  push_modrm(MOD_DIRECT, op.digit, reg_enc);
  push_imm(imm, size);
}

void Assembler::encode_memreg_regmem(Register reg,
                                     Memory mem,
                                     const Opcode& op,
                                     const InstructionEncoding& encoding) {
  const auto [reg_e, reg_enc] = get_register_encoding(reg);

  override_operand_size(encoding);
  encode_memory_operand(reg_enc, reg_e, get_rexw(encoding), op.op, mem);
}

void Assembler::encode_memimm(Memory mem,
                              Imm imm,
                              size_t size,
                              const OpcodeDigit& op,
                              const InstructionEncoding& encoding) {
  override_operand_size(encoding);
  encode_memory_operand(op.digit, false, get_rexw(encoding), op.op, mem);
  push_imm(imm, size);
}

void Assembler::encode_mem(Memory mem, const OpcodeDigit& op, const InstructionEncoding& encoding) {
  override_operand_size(encoding);
  encode_memory_operand(op.digit, false, get_rexw(encoding), op.op, mem);
}

void Assembler::encode_reg(Register reg,
                           const OpcodeDigit& op,
                           const InstructionEncoding& encoding) {
  const auto [reg_e, reg_enc] = get_register_encoding(reg);

  override_operand_size(encoding);
  push_rex(get_rexw(encoding), false, false, reg_e);
  push_bytes(op.op);
  push_modrm(MOD_DIRECT, op.digit, reg_enc);
}

void Assembler::encode_imm(Imm imm,
                           size_t size,
                           const Opcode& op,
                           const InstructionEncoding& encoding) {
  override_operand_size(encoding);
  push_rex(get_rexw(encoding), false, false, false);
  push_bytes(op.op);
  push_imm(imm, size);
}

void Assembler::encode_standalone(const Opcode& op, const InstructionEncoding& encoding) {
  override_operand_size(encoding);
  push_rex(get_rexw(encoding), false, false, false);
  push_bytes(op.op);
}

void Assembler::encode_rel32(int32_t rel,
                             Label* label,
                             const Opcode& op,
                             const InstructionEncoding& encoding) {
  X64_ASM_ASSERT(encoding.rexw == RexwMode::Unneeded && encoding.p66 == Prefix66Mode::Unneeded,
                 "Relative jumps/calls should not need REX or 66 prefix.");

  encode_imm(rel, sizeof(rel), op, encoding);

  if (label) {
    X64_ASM_ASSERT(rel == 0, "Target label was specified but relative offset was not 0.");

    const size_t end_offset = bytes.size();
    const size_t rel32_offset = end_offset - 4;

    fixups.push_back(Fixup{label, rel32_offset, end_offset});
  }
}

void Assembler::encode_regimm64(Register reg,
                                Imm imm,
                                const OpcodeRegadd& op,
                                const InstructionEncoding& encoding) {
  require_64bit();

  const auto [reg_e, reg_enc] = get_register_encoding(reg);

  X64_ASM_ASSERT(op.op.size == 1, "Only 1 byte opcodes for r64, imm64 are supported.");
  const uint8_t operation = op.op.bytes[0] + reg_enc;

  push_rex(true, false, false, reg_e);
  push_value(operation);
  push_imm(imm, 8);
}

const struct InstructionEncoding& Assembler::instruction_preprocess(
  std::string_view name,
  const FullInstructionEncoding& full_encoding,
  const Operand** operands,
  size_t operands_count) {
  const InstructionEncoding& encoding = [&]() -> const InstructionEncoding& {
    if (operand_size == OperandSize::Bits8) {
      X64_ASM_ASSERT(!!full_encoding.encoding_8bit,
                     "This instruction doesn't support 8 bit operand size.");
      X64_ASM_ASSERT(full_encoding.encoding_8bit->fix_8bit, "8 bit fix must be enabled.");

      return *full_encoding.encoding_8bit;
    }

    return full_encoding.encoding_normal;
  }();

  force_rex = false;

  // Make sure that we encode proper 8 bit register operands.
  if (encoding.fix_8bit) {
    for (size_t i = 0; i < operands_count; ++i) {
      if (const auto reg = operands[i]->get_reg()) {
        const auto r = *reg;

        // For this registers to be encoded with 8 bit size REX prefix
        // needs to be present.
        if (r == Register::Rsp || r == Register::Rbp || r == Register::Rsi || r == Register::Rdi) {
          force_rex = true;
          break;
        }
      }
    }
  }

  return encoding;
}

void Assembler::on_encoding_end() {
  if (pending_fixup_fill) {
    auto& fixup = fixups[fixups.size() - 1];

    X64_ASM_ASSERT(fixup.end_offset == 0, "Fixup fill is pending but end offset isn't 0.");

    fixup.end_offset = bytes.size();
    pending_fixup_fill = false;
  }
}

void Assembler::encode_0(std::string_view name, const FullInstructionEncoding& full_encoding) {
  EncodingGuard guard(this);

  const auto encoding = instruction_preprocess(name, full_encoding, nullptr, 0);

  if (encoding.standalone) {
    encode_standalone(*encoding.standalone, encoding);
    return;
  }

  X64_ASM_ASSERT(false, "This operand combination is unsupported for this instruction.");
}

void Assembler::encode_1(std::string_view name,
                         const FullInstructionEncoding& full_encoding,
                         const Operand& op0) {
  EncodingGuard guard(this);

  std::array operands{&op0};
  const auto encoding =
    instruction_preprocess(name, full_encoding, operands.data(), operands.size());

  switch (op0.get_type()) {
    case OpTy::Register:
      if (encoding.reg) {
        encode_reg(*op0.get_reg(), *encoding.reg, encoding);
        return;
      }
      break;

    case OpTy::Memory:
      if (encoding.mem) {
        encode_mem(*op0.get_memory(), *encoding.mem, encoding);
        return;
      }
      break;

    case OpTy::Label:
      if (encoding.rel32) {
        encode_rel32(0, get_label(*op0.get_label()), *encoding.rel32, encoding);
        return;
      }
      break;

    case OpTy::Immediate: {
      const auto imm = *op0.get_imm();

      if (fits_within<uint16_t>(imm) && encoding.uimm16) {
        encode_imm(imm, 2, *encoding.uimm16, encoding);
        return;
      }

      if (fits_within_imm32(imm) && encoding.imm32) {
        encode_imm(imm, get_imm32_size(), *encoding.imm32, encoding);
        return;
      }

      break;
    }

    default:
      break;
  }

  X64_ASM_ASSERT(false, "This operand combination is unsupported for this instruction.");
}

void Assembler::encode_2(std::string_view name,
                         const FullInstructionEncoding& full_encoding,
                         const Operand& op0,
                         const Operand& op1) {
  EncodingGuard guard(this);

  std::array operands{&op0, &op1};
  const auto encoding =
    instruction_preprocess(name, full_encoding, operands.data(), operands.size());

  constexpr auto get_combination = [](OpTy first, OpTy second) -> uint32_t {
    constexpr uint32_t code_shift = 8;
    return uint32_t(first) | (uint32_t(second) << code_shift);
  };

  switch (get_combination(op0.get_type(), op1.get_type())) {
    case get_combination(OpTy::Register, OpTy::Register):
      if (*op1.get_reg() == Register::Rcx && encoding.regcl) {
        encode_reg(*op0.get_reg(), *encoding.regcl, encoding);
        return;
      }
      if (encoding.regreg) {
        encode_regreg(*op0.get_reg(), *op1.get_reg(), *encoding.regreg, encoding);
        return;
      }
      if (encoding.regreg_inv) {
        encode_regreg(*op1.get_reg(), *op0.get_reg(), *encoding.regreg_inv, encoding);
        return;
      }
      break;

    case get_combination(OpTy::Register, OpTy::Memory):
      if (encoding.regmem) {
        encode_memreg_regmem(*op0.get_reg(), *op1.get_memory(), *encoding.regmem, encoding);
        return;
      }
      break;

    case get_combination(OpTy::Memory, OpTy::Register):
      if (*op1.get_reg() == Register::Rcx && encoding.memcl) {
        encode_mem(*op0.get_memory(), *encoding.memcl, encoding);
        return;
      }
      if (encoding.memreg) {
        encode_memreg_regmem(*op1.get_reg(), *op0.get_memory(), *encoding.memreg, encoding);
        return;
      }
      break;

    case get_combination(OpTy::Register, OpTy::Immediate): {
      const auto reg = *op0.get_reg();
      const auto imm = *op1.get_imm();

      if (fits_within<int8_t>(imm) && encoding.regimm8) {
        encode_regimm(reg, imm, 1, *encoding.regimm8, encoding);
        return;
      }

      if (fits_within<uint8_t>(imm) && encoding.reguimm8) {
        encode_regimm(reg, imm, 1, *encoding.reguimm8, encoding);
        return;
      }

      if (fits_within_imm32(imm) && encoding.regimm32) {
        encode_regimm(reg, imm, get_imm32_size(), *encoding.regimm32, encoding);
        return;
      }

      if (encoding.regimm64) {
        encode_regimm64(reg, imm, *encoding.regimm64, encoding);
        return;
      }

      break;
    }

    case get_combination(OpTy::Memory, OpTy::Immediate): {
      const auto mem = *op0.get_memory();
      const auto imm = *op1.get_imm();

      if (fits_within<int8_t>(imm) && encoding.memimm8) {
        encode_memimm(mem, imm, 1, *encoding.memimm8, encoding);
        return;
      }

      if (fits_within<uint8_t>(imm) && encoding.memuimm8) {
        encode_memimm(mem, imm, 1, *encoding.memuimm8, encoding);
        return;
      }

      if (fits_within_imm32(imm) && encoding.memimm32) {
        encode_memimm(mem, imm, get_imm32_size(), *encoding.memimm32, encoding);
        return;
      }

      break;
    }

    default:
      break;
  }

  X64_ASM_ASSERT(false, "This operand combination is unsupported for this instruction.");
}

void Assembler::apply_fixups() {
  for (const auto& fixup : fixups) {
    const auto label = fixup.label;
    X64_ASM_ASSERT(label->inserted, "Uninserted label was referenced.");

    const auto target = label->offset;

    using I32Limits = std::numeric_limits<int32_t>;

    // target = offset + instruction_size + rel32
    // rel32  = target - offset - instruction_size
    // rel32 = target - end_offset
    const int64_t rel64 = int64_t(target) - int64_t(fixup.end_offset);

    X64_ASM_ASSERT(rel64 >= int64_t(I32Limits::min()) && rel64 <= int64_t(I32Limits::max()),
                   "Cannot encode jump target in rel32.");

    const auto rel32 = uint32_t(rel64);
    size_t write_offset = fixup.rel32_offset;

    for (const auto byte : std::bit_cast<std::array<uint8_t, sizeof(rel32)>>(rel32)) {
      bytes[write_offset++] = byte;
    }
  }

  fixups.clear();
}

std::span<const uint8_t> Assembler::get_assembled_bytes() {
  apply_fixups();
  return bytes;
}

Label* Assembler::create_label() {
  return label_pool.allocate();
}

Label* Assembler::label() {
  const auto label_ = create_label();
  label(label_);
  return label_;
}

void Assembler::label(Label* label) {
  X64_ASM_ASSERT(!label->inserted, "Given label was already inserted.");
  label->offset = bytes.size();
  label->inserted = true;
}

void Assembler::label(std::string label_name) {
  label(get_or_create_named_label(std::move(label_name)));
}

}  // namespace asmlib::x64