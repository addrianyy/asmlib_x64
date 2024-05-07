#pragma once
#include <optional>
#include <string>
#include <variant>

namespace asmlib::x64 {

enum class Register {
  Rax,
  Rbx,
  Rcx,
  Rdx,
  Rsp,
  Rbp,
  Rsi,
  Rdi,
  R8,
  R9,
  R10,
  R11,
  R12,
  R13,
  R14,
  R15,
};

using Imm = int64_t;

class Label {
  friend class Assembler;

  uint64_t inserted : 1 = 0;
  uint64_t offset : 63 = 0;
};

namespace detail {
struct LabelOperand {
  Label* label_ = nullptr;
  std::string_view name_;

  static LabelOperand label(Label* label) {
    return LabelOperand{.label_ = label, .name_ = std::string_view()};
  }
  static LabelOperand name(std::string_view name) {
    return LabelOperand{.label_ = nullptr, .name_ = name};
  }
};
}  // namespace detail

class Memory {
  friend class Assembler;

 public:
  struct IndexScale {
    Register index;
    uint32_t scale;
  };

 private:
  std::optional<Register> base_;
  std::optional<IndexScale> index_scale_;
  int32_t displacement_ = 0;

  std::optional<detail::LabelOperand> label_;

  explicit Memory(std::optional<Register> base,
                  std::optional<IndexScale> index_scale = std::nullopt,
                  int32_t displacement = 0)
      : base_(base), index_scale_(index_scale), displacement_(displacement) {}

  explicit Memory(detail::LabelOperand label) : label_(label) {}

  std::optional<Register> get_base() const { return base_; }
  std::optional<IndexScale> get_index() const { return index_scale_; }
  int32_t get_displacement() const { return displacement_; }

  std::optional<detail::LabelOperand> get_label() const { return label_; }

 public:
  static inline Memory label(std::string_view label_name) {
    return Memory(detail::LabelOperand::name(label_name));
  }
  static inline Memory label(Label* label) { return Memory(detail::LabelOperand::label(label)); }

  static inline Memory base(Register base) { return Memory(base); }
  static inline Memory index(Register index, uint32_t scale) {
    return Memory(std::nullopt, IndexScale{index, scale});
  }
  static inline Memory disp(int32_t displacement) {
    return Memory(std::nullopt, std::nullopt, displacement);
  }
  static inline Memory base_index(Register base, Register index, uint32_t scale) {
    return Memory(base, IndexScale{index, scale});
  }
  static inline Memory base_disp(Register base, int32_t displacement) {
    return Memory(base, std::nullopt, displacement);
  }
  static inline Memory index_disp(Register index, uint32_t scale, int32_t displacement) {
    return Memory(std::nullopt, IndexScale{index, scale}, displacement);
  }
  static inline Memory base_index_disp(Register base,
                                       Register index,
                                       uint32_t scale,
                                       int32_t displacement) {
    return Memory(base, IndexScale{index, scale}, displacement);
  }
};

class Operand {
  friend class Assembler;

 public:
  enum class Type {
    Register,
    Immediate,
    Memory,
    Label,
  };

 private:
  std::variant<Register, Imm, Memory, detail::LabelOperand> op;
  Type type;

  const Register* get_reg() const { return std::get_if<Register>(&op); }
  const Imm* get_imm() const { return std::get_if<Imm>(&op); }
  const Memory* get_memory() const { return std::get_if<Memory>(&op); }
  const detail::LabelOperand* get_label() const {
    if (const auto label = std::get_if<detail::LabelOperand>(&op)) {
      return label;
    }
    return nullptr;
  }

  Type get_type() const { return type; }

 public:
  Operand(Register reg) : op(reg), type(Type::Register) {}
  Operand(Imm imm) : op(imm), type(Type::Immediate) {}
  Operand(Label* label) : op(detail::LabelOperand::label(label)), type(Type::Label) {}
  Operand(std::string_view label) : op(detail::LabelOperand::name(label)), type(Type::Label) {}
  Operand(const char* label) : op(detail::LabelOperand::name(label)), type(Type::Label) {}
  Operand(Memory memory) : op(memory), type(Type::Memory) {}

  static inline Operand reg(Register reg) { return Operand(reg); }
  static inline Operand imm(Imm imm) { return Operand(imm); }
  static inline Operand label(Label* label) { return Operand(label); }
  static inline Operand label(std::string_view label) { return Operand(label); }
  static inline Operand memory(Memory memory) { return Operand(memory); }
};

}  // namespace asmlib::x64