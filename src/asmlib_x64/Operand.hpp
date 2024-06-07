#pragma once
#include <cstdint>
#include <limits>
#include <optional>

namespace asmlib::x64 {

enum class Register : uint8_t {
  Rax,
  Rcx,
  Rdx,
  Rbx,
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

namespace detail {

constexpr Register invalid_register =
  Register(std::numeric_limits<std::underlying_type_t<Register>>::max());

}

using Imm = int64_t;

class Label {
  friend class Assembler;

  constexpr static uint32_t invalid_index = std::numeric_limits<uint32_t>::max();
  uint32_t index{invalid_index};

  explicit Label(uint32_t index) : index(index) {}

 public:
  Label() = default;

  operator bool() const { return index != invalid_index; }
};

class Memory {
  friend class Assembler;

 public:
  struct IndexScale {
    Register index;
    uint32_t scale;
  };

 private:
  Register base_ = detail::invalid_register;
  Register index_ = detail::invalid_register;
  uint8_t scale_ = 0;
  int32_t displacement_ = 0;

  Label label_;

  explicit Memory(std::optional<Register> base,
                  std::optional<IndexScale> index_scale = std::nullopt,
                  int32_t displacement = 0);

  explicit Memory(Label label);

  std::optional<Register> get_base() const;
  std::optional<IndexScale> get_index() const;
  int32_t get_displacement() const;
  Label get_label() const;

 public:
  static Memory label(Label label) { return Memory(label); }

  static Memory base(Register base) { return Memory(base); }
  static Memory index(Register index, uint32_t scale) {
    return Memory(std::nullopt, IndexScale{index, scale});
  }
  static Memory disp(int32_t displacement) {
    return Memory(std::nullopt, std::nullopt, displacement);
  }
  static Memory base_index(Register base, Register index, uint32_t scale) {
    return Memory(base, IndexScale{index, scale});
  }
  static Memory base_disp(Register base, int32_t displacement) {
    return Memory(base, std::nullopt, displacement);
  }
  static Memory index_disp(Register index, uint32_t scale, int32_t displacement) {
    return Memory(std::nullopt, IndexScale{index, scale}, displacement);
  }
  static Memory base_index_disp(Register base,
                                Register index,
                                uint32_t scale,
                                int32_t displacement) {
    return Memory(base, IndexScale{index, scale}, displacement);
  }
};

class Operand {
  friend class Assembler;

 public:
  enum class Type : uint8_t {
    Register,
    Immediate,
    Memory,
    Label,
  };

 private:
  static_assert(std::is_trivially_destructible_v<Register> &&
                  std::is_trivially_destructible_v<Imm> &&
                  std::is_trivially_destructible_v<Memory> &&
                  std::is_trivially_destructible_v<Label>,
                "operand types must be trivially destructible");

  union OperandContainer {
    Register reg;
    Imm imm;
    Memory mem;
    Label label;

    explicit OperandContainer(Register reg) : reg(reg) {}
    explicit OperandContainer(Imm imm) : imm(imm) {}
    explicit OperandContainer(Memory mem) : mem(mem) {}
    explicit OperandContainer(Label label) : label(label) {}
  };

  Type type;
  OperandContainer op;

  const Register* get_reg() const;
  const Imm* get_imm() const;
  const Memory* get_memory() const;
  const Label* get_label() const;

  Type get_type() const { return type; }

 public:
  Operand(Register reg);
  Operand(Imm imm);
  Operand(Label label);
  Operand(Memory memory);

  static Operand reg(Register reg) { return Operand(reg); }
  static Operand imm(Imm imm) { return Operand(imm); }
  static Operand label(Label label) { return Operand(label); }
  static Operand memory(Memory memory) { return Operand(memory); }
};

}  // namespace asmlib::x64