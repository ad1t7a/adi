#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>

#ifdef RBDL_USE_ROS_URDF_LIBRARY
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef const boost::shared_ptr<const urdf::Link> ConstLinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<urdf::ModelInterface> ModelPtr;

#else
#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>

typedef my_shared_ptr<urdf::Link> LinkPtr;
typedef const my_shared_ptr<const urdf::Link> ConstLinkPtr;
typedef my_shared_ptr<urdf::Joint> JointPtr;

#endif

namespace RigidBodyDynamics {

struct Model;
typedef my_shared_ptr<urdf::ModelInterface> ModelPtr;
namespace Addons {
RBDL_DLLAPI bool URDFReadFromFile(const char *filename, Model *model,
                                  ModelPtr &urdfModel, bool floating_base,
                                  bool verbose = false);
RBDL_DLLAPI bool URDFReadFromString(const char *model_xml_string, Model *model,
                                    ModelPtr &urdfModel, bool floating_base,
                                    bool verbose = false);
} // namespace Addons

} // namespace RigidBodyDynamics

/* _RBDL_URDFREADER_H */
#endif
