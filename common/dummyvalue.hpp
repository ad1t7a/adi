#pragma once

#include <limits>
namespace adi {
/*
This provides dummy values for scalars - a value that is unlikey to be mistaken
for its purposefully computed value, useful for initializing before its true
result is available.

It defaults to std::numeric_limits::quiet_NaN when availabe ; it is a compile
time error to call the unspecialized dummy_value::get() when quiet_NaN is
unavailable
*/
template <typename T> struct DummyValue {
  static constexpr T get() {
    static_assert(std::numeric_limits<T>::has_quiet_NaN,
                  "Custom scalar types should specialize this struct");
    return std::numeric_limits<T>::quiet_NaN();
  }
};

template <> struct DummyValue<int> {
  static constexpr int get() { return 0xDDDDDDDD; }
};
} // namespace adi