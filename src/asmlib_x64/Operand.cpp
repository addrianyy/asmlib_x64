#include "Operand.hpp"

namespace asmlib::x64 {

Memory::Memory(std::optional<Register> base,
               std::optional<IndexScale> index_scale,
               int32_t displacement)
    : displacement_(displacement) {
  if (base) {
    base_ = *base;
  }

  if (index_scale) {
    index_ = index_scale->index;
    scale_ = index_scale->scale;
  }
}

Memory::Memory(Label label) : label_(label) {}

std::optional<Register> Memory::get_base() const {
  return base_ == detail::invalid_register ? std::nullopt : std::optional(base_);
}

std::optional<Memory::IndexScale> Memory::get_index() const {
  if (index_ != detail::invalid_register) {
    return IndexScale{
      .index = index_,
      .scale = scale_,
    };
  }

  return std::nullopt;
}

int32_t Memory::get_displacement() const {
  return displacement_;
}

Label Memory::get_label() const {
  return label_;
}

const Register* Operand::get_reg() const {
  return type == Type::Register ? &op.reg : nullptr;
}
const Imm* Operand::get_imm() const {
  return type == Type::Immediate ? &op.imm : nullptr;
}
const Memory* Operand::get_memory() const {
  return type == Type::Memory ? &op.mem : nullptr;
}
const Label* Operand::get_label() const {
  return type == Type::Label ? &op.label : nullptr;
}

Operand::Operand(Register reg) : type(Type::Register), op(reg) {}
Operand::Operand(Imm imm) : type(Type::Immediate), op(imm) {}
Operand::Operand(Label label) : type(Type::Label), op(label) {}
Operand::Operand(Memory memory) : type(Type::Memory), op(memory) {}

}  // namespace asmlib::x64