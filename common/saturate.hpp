#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"

namespace adi {
namespace math {
template <typename T1, typename T2, typename T3>
T1 saturate(const T1 &value, const T2 &low, const T3 &high) {
  ADI_ASSERT(low < high);
  return cond(value<low, low, value> high, high, value);
}
} // namespace math
} // namespace adi