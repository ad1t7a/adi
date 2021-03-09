#pragma once

#include <type_traits>

#include "adi_assert.hpp"

namespace adi {
namespace detail {
// Throw an error message.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Throw(const char* condition, const char* func, const char* file, int line);
} // namespace detail
} // namespace adi

/// Evaluates @p condition and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
#define ADI_THROW_UNLESS(condition)                                            \
  do {                                                                         \
    typedef ::adi::assert::ConditionTraits<                                    \
        typename std::remove_cv<decltype(condition)>::type>                    \
        Trait;                                                                 \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");   \
    if (!Trait::Evaluate(condition)) {                                         \
      ::adi::detail::Throw(#condition, __func__, __FILE__, __LINE__);          \
    }                                                                          \
  } while (0)