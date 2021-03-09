#pragma once

#include <iostream>
#include <numeric>
#include <vector>

namespace adi {
/*
argument sort and return indices of sorted vector
*/
template <typename T> std::vector<size_t> arg_sort(const std::vector<T> &vec) {
  std::vector<size_t> idx(vec.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(),
            [&vec](size_t i0, size_t i1) { return vec[i0] < vec[i1]; });
  return idx;
}

} // namespace adi