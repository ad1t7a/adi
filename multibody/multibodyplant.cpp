#include "multibody/multibodyplant.hpp"

namespace adi {
namespace physics {
//! constructor
MultiBodyPlant::MultiBodyPlant(std::string urdfPath, bool floatingBase) {
  m_urdfPath = urdfPath;
  m_floatingBase = floatingBase;
}

//! destructor
MultiBodyPlant::~MultiBodyPlant() {}

} // namespace physics
} // namespace adi