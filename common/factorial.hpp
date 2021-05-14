#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"

namespace adi {
template <auto N> struct Factorial {
  static constexpr auto value = N * Factorial<N - 1>::value;
};

template <> struct Factorial<0> { static constexpr auto value = 1; };

} // namespace adi