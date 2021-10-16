#pragma once
#include <string>

namespace detail {
void asm_assert_internal(bool value, const char* message, const char* file, int line);
inline void asm_assert_internal(bool value, const std::string& message, const char* file,
                                int line) {
  asm_assert_internal(value, message.c_str(), file, line);
}
} // namespace detail

#define asm_assert(value, message)                                                                 \
  ::detail::asm_assert_internal((value), (message), __FILE__, __LINE__)