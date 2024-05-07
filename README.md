# x64 assembler library in C++20

```cpp
using namespace asmlib::x64;

Assembler as;

const auto test_label = as.allocate_label();
const auto other_label = as.allocate_label();

as.rdtsc();
as.nop();
as.insert_label(test_label);
as.int3();
as.inc(Register::Rsp);
as.neg(Register::Rdi);
as.ja(test_label);
as.ret(100);
as.neg(Memory::base_index_disp(Register::Rax, Register::Rcx, 4, 0x123));
as.with_operand_size(OperandSize::Bits8, [&]() {
as.neg(Memory::base(Register::Rdi));
as.neg(Register::Rbx);
});
as.mov(Memory::base(Register::Rdx), Register::Rcx);
as.mov(Register::R12, Memory::base(Register::Rdx));
as.add(Register::Rdx, Register::Rdi);
as.insert_label(other_label);
as.shr(Memory::base(Register::Rdx), Register::Rcx);
as.mov(Memory::label(test_label), 55);
as.add(Register::Rdx, 448);
as.mov(Register::Rdx, 1235678918881ll);
as.call(other_label);
```

```asm
00000000  0F31              rdtsc
00000002  90                nop
00000003  CC                int3
00000004  48FFC4            inc rsp
00000007  48F7DF            neg rdi
0000000A  0F87F3FFFFFF      ja near 0x3
00000010  C26400            ret 0x64
00000013  48F79C8823010000  neg qword [rax+rcx*4+0x123]
0000001B  F61F              neg byte [rdi]
0000001D  F6DB              neg bl
0000001F  48890A            mov [rdx],rcx
00000022  4C8B22            mov r12,[rdx]
00000025  4803D7            add rdx,rdi
00000028  48D32A            shr qword [rdx],cl
0000002B  48C705CDFFFFFF37  mov qword [rel 0x3],0x37
         -000000
00000036  4881C2C0010000    add rdx,0x1c0
0000003D  48BAE1F833B41F01  mov rdx,0x11fb433f8e1
         -0000
00000047  E8DCFFFFFF        call 0x28
```