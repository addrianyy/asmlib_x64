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
  return index_ == detail::invalid_register ? std::nullopt : std::optional(index_);
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

}  // namespace asmlib::x64