#include "visualization/pointcloud.hpp"
#include "visualization/urdf.hpp"

int main() {
  size_t numRobots = 1;
  std::vector<adi::visualization::URDF*> vectorURDF;
  for(size_t k=0; k<numRobots; k++) {
    adi::visualization::URDF* urdf = new
  adi::visualization::URDF(std::string("tcp://127.0.0.1:6000"));
    urdf->setPathPrefix("/Users/ad1t7a/Developer/adi/robots/ur5/");
    urdf->loadURDF("robot.urdf", true, "texture.jpg", std::to_string(k));
    vectorURDF.push_back(urdf);
  }

  RigidBodyDynamics::Math::VectorNd Q =
      RigidBodyDynamics::Math::VectorNd::Zero(vectorURDF[0]->getDoF());
  std::cout << "Q[3] = " << vectorURDF[0]->getDoF() << "\n";
  double t = 0.0;
  while (true) {
    for(size_t k=0; k<vectorURDF.size(); k++) {
      // translation position
      Q[0] = 0;
      Q[1] = 0;
      Q[2] = 0.0;
      Q[3] = 0;
      Q[4] = 0;
      Q[5] = 0.0;
      vectorURDF[k]->syncVisualTransforms(Q);
    }
    t += 0.001;
  }

  //! end effector visualization
  /*adi::visualization::URDF endEffector(std::string("tcp://127.0.0.1:6000"));
  endEffector.setPathPrefix("/Users/ad1t7a/Developer/adi/robots/objects/");
  endEffector.loadURDF("003_cracker_box_textured.urdf", false, "meshes/003_cracker_box_textured.png");
  std::vector<RigidBodyDynamics::Math::SpatialTransform> transforms;
  RigidBodyDynamics::Math::SpatialTransform transform;
  transform.r ={0.5, -0.3, 0.5};
  transform.E ={-1.0000000,  0.0000000, -0.0000000, 0.0000000,  1.0, -0.0, 0.0000000, -0.0, -1.0};
  transforms.push_back(transform);
  endEffector.syncVisualTransforms(transforms);*/

  /*adi::visualization::PointCloud pc(std::string("tcp://127.0.0.1:6000"));
  pc.loadPointCloud("/adi/test");*/

  return 0;
}