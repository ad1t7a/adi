#include "multibody/multibodyplant.hpp"

namespace adi {
namespace multibody {
//! constructor
MultiBodyPlant::MultiBodyPlant(std::string urdfPath, bool floatingBase) {
  m_urdfPath = urdfPath;
  m_floatingBase = floatingBase;
}

//! destructor
MultiBodyPlant::~MultiBodyPlant() {}

} // namespace multibody
} // namespace adi