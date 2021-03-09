#pragma once

namespace adi {

/* Provides if_then_else expression for double. The value returned by the
if_then_else expression is v_then if f_cond is true. Otherwise, it
returns v_else. */
inline double if_then_else(bool f_cond, double v_then, double v_else) {
  return f_cond ? v_then : v_else;
}

} // namespace adi