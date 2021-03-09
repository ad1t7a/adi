#pragma once

#include <new>
#include <type_traits>
#include <utility>

#include "copyable.hpp"

namespace adi {

template <typename T> class never_destroyed {
public:
  ADI_NO_COPY_NO_MOVE_NO_ASSIGN(never_destroyed)

  /// Passes the constructor arguments along to T using perfect forwarding.
  template <typename... Args> explicit never_destroyed(Args &&...args) {
    // Uses "placement new" to construct a `T` in `storage_`.
    new (&storage_) T(std::forward<Args>(args)...);
  }

  /// Does nothing.  Guaranteed!
  ~never_destroyed() = default;

  /// Returns the underlying T reference.
  T &access() { return *reinterpret_cast<T *>(&storage_); }
  const T &access() const { return *reinterpret_cast<const T *>(&storage_); }

private:
  typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
};

} // namespace adi