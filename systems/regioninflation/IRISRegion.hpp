#pragma once

#include "common/copyable.hpp"
#include "mathematics/ellipsoid.hpp"
#include "mathematics/polyhedron.hpp"

namespace adi {
namespace systems {

class IRISRegion {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IRISRegion);

  //! constructor
  IRISRegion(int dim) : mPolyhedron(dim), mEllipsoid(dim) {}

  //! destructor
  ~IRISRegion() {}

  //! get polyhedron
  adi::mathematics::Polyhedron getPolyhedron() { return mPolyhedron; }

  //! get polyhedron
  adi::mathematics::Ellipsoid getEllipsoid() { return mEllipsoid; }

  //! polyhedron
  adi::mathematics::Polyhedron mPolyhedron;

  //! ellipsoid
  adi::mathematics::Ellipsoid mEllipsoid;
};
} // namespace systems
} // namespace adi