#pragma once

#include "spdlog/spdlog.h"
#include <type_traits>

#ifdef ADI_DOXYGEN
/*
Assertion implementation. This is intended to be used with all implementation
within and outside this library. The assertions can be armed and disarmed
independently of the default c++ assertions.
*/

/*
The asserts are similar to built-in assert functions except that they can be
disarmed by preprocessor definitions listed below. If the asserts are armed, the
condition is evaluated and if evaluated as false, will trigger an assertion
failure showing the message, line and function name.

The assertions are enabled/disabled using the following macro:
    @ ADI_ENABLE_ASSERT  : Arms the assert functions
    @ ADI_DISABLE_ASSERT : Disarms the assert functions
    If both are set, it leads to a compile-time error
*/

/*
Treat asserts like a statement; use them within some scope and end with a
semi-colon
*/
#define ADI_ASSERT(condition)

/*
Similar to ADI_ASSERT except that the expresssion to be evaluated has to be void
valued This allows for guarding assertion checking
*/
#define ADI_ASSERT_VOID(expression)

/*
Evaluates a condition and if the value is false, will trigger assertion failure
with a message showing at least the name and line of the function.
*/
#define ADI_DEMAND(condition)

/*
Abort the program showing the name and file where the abort was called. This
abort is done using the ::abort function.
*/
#define ADI_ABORT()

/*
Abort the program showing a message apart from the name and file where the abort
was called. This abort is done using the ::abort function.
*/
#define ADI_ABORT_MSG()
#else

#ifdef ADI_ASSERT_IS_ARMED
#error Unexpected ADI_ASSERT_IS_ARMED defined.
#endif

#ifdef ADI_ASSERT_IS_DISARMED
#error Unexpected ADI_ASSERT_IS_DISARMED defined.
#endif

// Decide whether assertions are enabled
#if defined(ADI_ENABLE_ASSERT) && defined(ADI_DISABLE_ASSERT)
#error Conflicting assertions
#elif defined(ADI_ENABLE_ASSERT)
#define ADI_ASSERT_IS_ARMED
#elif defined(ADI_DISABLE_ASSERT)
#define ADI_ASSERT_IS_DISARMED
#else
#define ADI_ASSERT_IS_ARMED
#endif

namespace adi {
namespace detail {
/* Abort the program with an error message*/
__attribute__((noreturn)) void Abort(const char *condition, const char *func,
                                     const char *file, int line);

__attribute__((noreturn)) void AssertionFailed(const char *condition,
                                               const char *func,
                                               const char *file, int line);
} // namespace detail

namespace assert {
template <typename Condition> struct ConditionTraits {
  static constexpr bool is_valid = std::is_convertible<Condition, bool>::value;
  static bool Evaluate(const Condition &value) { return value; }
};
} // namespace assert
} // namespace adi

#define ADI_ABORT() ::adi::detail::Abort(nullptr, __func__, __FILE__, __LINE__)

#define ADI_DEMAND(condition)                                                  \
  do {                                                                         \
    typedef ::adi::assert::ConditionTraits<                                    \
        typename std::remove_cv<decltype(condition)>::type>                    \
        Trait;                                                                 \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");   \
    if (!Trait::Evaluate(condition)) {                                         \
      ::adi::detail::AssertionFailed(#condition, __func__, __FILE__,           \
                                     __LINE__);                                \
    }                                                                          \
  } while (0)

#define ADI_ABORT_MSG(msg)                                                     \
  ::adi::detail::Abort(msg, __func__, __FILE__, __LINE__)

#ifdef ADI_ASSERT_IS_ARMED

#define ADI_ASSERT(condition) ADI_DEMAND(condition)

#define ADI_ASSERT_VOID(expression)                                            \
  do {                                                                         \
    static_assert(std::is_convertible<decltype(expression), void>::value,      \
                  "Expression should be void.");                               \
    expression;                                                                \
  } while (0)
#else
// Assertions are disabled, so just typecheck the expression.
#define ADI_ASSERT(condition)                                                  \
  static_assert(                                                               \
      ::adi::assert::ConditionTraits<                                          \
          typename std::remove_cv<decltype(condition)>::type>::is_valid,       \
      "Condition should be bool-convertible.");
#define ADI_ASSERT_VOID(expression)                                            \
  static_assert(std::is_convertible<decltype(expression), void>::value,        \
                "Expression should be void.")
#endif

#endif // ADI_DOXYGEN
