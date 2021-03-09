#pragma once

#include <stdexcept>
#include <string>

namespace adi {
namespace detail {

/// This is what ADI_ASSERT and ADI_DEMAND throw when our assertions are
/// configured to throw.
class assertion_error : public std::runtime_error {
public:
  explicit assertion_error(const std::string &what_arg)
      : std::runtime_error(what_arg) {}
};

} // namespace detail
} // namespace adi