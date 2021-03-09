#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"
namespace adi {
namespace systems {
class IRISOptions {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IRISOptions);

  //! constructor
  IRISOptions()
      : requireContainment(false), errorOnInfeasibleStart(false),
        terminationThreshold(2e-2), iterLimit(100),
        requiredContainmentPoints({}) {}

  //! destructor
  ~IRISOptions() {}

  // If require_containment is true and required_containment_points is empty,
  // then the IRIS region is required to contain the center of the seed
  // ellipsoid. Otherwise, the IRIS region is required to contain all points
  // in required_containment_points.
  // If require_containment is false, then required_containment_points has no
  // effect.
  bool requireContainment;

  //! error on infeasibility
  bool errorOnInfeasibleStart;

  //! termination threshold
  double terminationThreshold;

  //! iteration limit
  int iterLimit;

  //! containment points
  std::vector<VectorXd> requiredContainmentPoints;
};
} // namespace systems
} // namespace adi