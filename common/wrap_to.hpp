#pragma once

#include <cmath>

#include "common/adi_assert.hpp"

namespace adi {
namespace math {

template <typename T1, typename T2>
T1 wrap_to(const T1 &value, const T2 &low, const T2 &high) {
  ADI_ASSERT(low < high);
  const T2 range = high - low;
  return value - range * floor((value - low) / range);
}
} // namespace math
} // namespace adi
