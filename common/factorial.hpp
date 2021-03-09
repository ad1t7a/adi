#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"

namespace adi {
namespace math {
template <typename T> T factorial(T n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}
} // namespace math
} // namespace adi