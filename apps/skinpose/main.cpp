#include "visualization/pointcloud.hpp"
#include "visualization/urdf.hpp"

int main() {
    std::string objectPath = "";
    std::string pointCloudPath = "";

    //! object visualization
    adi::visualization::URDF datum(std::string("tcp://127.0.0.1:6000"));
    datum.setPathPrefix("/Users/ad1t7a/Developer/adi/robots/robotiq/");
    datum.loadURDF("gripper.urdf", false, "texture.jpg");
    std::vector<RigidBodyDynamics::Math::SpatialTransform> transforms;
    RigidBodyDynamics::Math::SpatialTransform transform;
    transform.r ={0.0, 0.0, 0.0};
    transform.E ={1.0000000,  0.0000000, 0.0000000, 0.0000000,  1.0000000, 0.0000000, 0.0000000, 0.000000, 1.00000};
    transforms.push_back(transform);
    datum.syncVisualTransforms(transforms);

    /*adi::visualization::PointCloud pc(std::string("tcp://127.0.0.1:6000"));
    pc.loadPointCloud("/adi/test");*/

    return 0;
}