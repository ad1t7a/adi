#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"
#include "mathematics/ellipsoid.hpp"
#include "mathematics/polyhedron.hpp"
#include "systems/regioninflation/IRISOptions.hpp"
#include "systems/regioninflation/IRISProblem.hpp"
#include "systems/regioninflation/IRISRegion.hpp"

namespace adi {
namespace systems {
class IRIS {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IRIS);
  //! constructor
  IRIS() {}

  //! destructor
  ~IRIS() {}

  //! region inflation algrithm
  IRISRegion inflateRegion(IRISProblem problem, IRISOptions options);

private:
  //! separating hyperplane
  void separatingHyperplanes(const std::vector<MatrixXd> obstaclePts,
                             const adi::mathematics::Ellipsoid &ellipsoid,
                             adi::mathematics::Polyhedron &polyhedron,
                             bool &infeasibleStart);
};
} // namespace systems
} // namespace adi