#pragma once

#include "doubleoverloads.hpp"
#include <functional>
#include <type_traits>

namespace adi {
/*
Constructs conditional expression
cond(cond_1, expr_1,
     cond_2, expr_2,
     cond_3, expr_3,
     cond_4, expr_4
     expr(n+1))
The value returned by the cond expression is expr_1 if cond_1 is true;
else expr_2 if cond_2 is true. If none of the conditions are true, it would
return expr(n+1)
*/
template <typename ScalarType> ScalarType cond(const ScalarType &e) {
  return e;
}

template <typename ScalarType, typename... Rest>
ScalarType cond(const decltype(ScalarType() < ScalarType()) &f_cond,
                const ScalarType &e_then, Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}
} // namespace adi