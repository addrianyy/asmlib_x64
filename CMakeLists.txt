cmake_minimum_required(VERSION 3.8)
project(asmlib)

set(CMAKE_CXX_STANDARD 20)

add_library(asmlib
        asmlib/Assembler.cpp
        asmlib/Assembler.hpp
        asmlib/Operand.cpp
        asmlib/Operand.hpp
        asmlib/private/AssemblerEncoding.cpp
        asmlib/private/AssemblerEncoding.hpp
        asmlib/private/Assert.cpp
        asmlib/private/Assert.hpp)
target_include_directories(asmlib PUBLIC .)