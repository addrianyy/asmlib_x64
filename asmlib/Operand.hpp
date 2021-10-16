#pragma once
#include <optional>
#include <string>
#include <variant>

namespace asmlib {

enum class Reg {
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

class Memory {
public:
  struct IndexScale {
    Reg index;
    uint32_t scale;
  };

private:
  std::optional<Reg> base_;
  std::optional<IndexScale> index_scale_;
  int32_t displacement_ = 0;

  std::string_view label_;

  explicit Memory(std::optional<Reg> base, std::optional<IndexScale> index_scale = std::nullopt,
                  int32_t displacement = 0)
      : base_(base), index_scale_(index_scale), displacement_(displacement) {}

  explicit Memory(std::string_view label_name) : label_(label_name) {}

public:
  static inline Memory label(std::string_view label_name) { return Memory(label_name); }

  static inline Memory base(Reg base) { return Memory(base); }
  static inline Memory index(Reg index, uint32_t scale) {
    return Memory(std::nullopt, IndexScale{index, scale});
  }
  static inline Memory disp(int32_t displacement) {
    return Memory(std::nullopt, std::nullopt, displacement);
  }
  static inline Memory base_index(Reg base, Reg index, uint32_t scale) {
    return Memory(base, IndexScale{index, scale});
  }
  static inline Memory base_disp(Reg base, int32_t displacement) {
    return Memory(base, std::nullopt, displacement);
  }
  static inline Memory index_disp(Reg index, uint32_t scale, int32_t displacement) {
    return Memory(std::nullopt, IndexScale{index, scale}, displacement);
  }
  static inline Memory base_index_disp(Reg base, Reg index, uint32_t scale, int32_t displacement) {
    return Memory(base, IndexScale{index, scale}, displacement);
  }

  std::optional<Reg> get_base() const { return base_; }
  std::optional<IndexScale> get_index() const { return index_scale_; }
  int32_t get_displacement() const { return displacement_; }

  std::string_view get_label() const { return label_; }
};

class Operand {
public:
  enum class Type {
    Register,
    Immediate,
    Memory,
    Label,
  };

private:
  struct Label {
    std::string_view name;
  };

  std::variant<Reg, Imm, Memory, Label> op;
  Type type;

public:
  Operand(Reg reg) : op(reg), type(Type::Register) {}
  Operand(Imm imm) : op(imm), type(Type::Immediate) {}
  Operand(std::string_view label) : op(Label{label}), type(Type::Label) {}
  Operand(const char* label) : op(Label{std::string_view(label)}), type(Type::Label) {}
  Operand(Memory memory) : op(memory), type(Type::Memory) {}

  static inline Operand reg(Reg reg) { return Operand(reg); }
  static inline Operand imm(Imm imm) { return Operand(imm); }
  static inline Operand label(std::string_view label) { return Operand(label); }
  static inline Operand memory(Memory memory) { return Operand(memory); }

  const Reg* get_reg() const { return std::get_if<Reg>(&op); }
  const Imm* get_imm() const { return std::get_if<Imm>(&op); }
  const Memory* get_memory() const { return std::get_if<Memory>(&op); }
  const std::string_view* get_label() const {
    if (const auto label = std::get_if<Label>(&op)) {
      return &label->name;
    }
    return nullptr;
  }

  Type get_type() const { return type; }
};

} // namespace asmlib