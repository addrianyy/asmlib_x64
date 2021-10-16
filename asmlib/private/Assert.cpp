#include "Assert.hpp"
#include <iostream>

void detail::asm_assert_internal(bool value, const char* message, const char* file, int line) {
  if (!value) {
    std::cerr << "Assertion failed: " << message << std::endl;
    std::cerr << file << ":" << line << std::endl;

    std::exit(1);
  }
}