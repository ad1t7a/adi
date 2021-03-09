#include "systems/fastplanner/sweptvolume.hpp"
#include "visualization/pointcloud.hpp"
#include "visualization/urdf.hpp"

int main() {

  /*size_t numRobots = 1;
  std::vector<adi::visualization::URDF*> vectorURDF;
  for(size_t k=0; k<numRobots; k++) {
    adi::visualization::URDF* urdf = new
  adi::visualization::URDF(std::string("tcp://127.0.0.1:6000"));
    urdf->setPathPrefix("/Users/ad1t7a/Developer/adi/robots/flexiv/");
    urdf->loadURDF("robot.urdf", false, "texture.jpg", std::to_string(k));
    vectorURDF.push_back(urdf);
  }


  RigidBodyDynamics::Math::VectorNd Q =
      RigidBodyDynamics::Math::VectorNd::Zero(vectorURDF[0]->getDoF());
  double t = 0.0;
  while (true) {
    Q[0] = 0.75 * sin(t);
    Q[1] = 0.75 * sin(t);
    Q[2] = 0.75 * sin(t);
    Q[3] = 1.5 * sin(t);
    Q[4] = 0.5 * sin(t);
    Q[5] = 0.5 * sin(t);
    Q[6] = 0.5 * sin(t);
    for(size_t k=0; k<vectorURDF.size(); k++) {
      vectorURDF[k]->syncVisualTransforms(Q, {(double)(k/10), (double)(k%10),
  0.0});
    }
    t += 0.001;
  }*/

  /*adi::systems::SweptVolume sw("/Users/ad1t7a/Developer/adi/robots/flexiv/",
  "robot.urdf", false); std::vector<adi::Vector3d> points; points =
  sw.calculateSweptVolume(RigidBodyDynamics::Math::VectorNd::Zero(sw.getDoF()),
  0.001*RigidBodyDynamics::Math::VectorNd::Ones(sw.getDoF())); for(size_t i=0;
  i< points.size(); i++) { std::cout << points[i].transpose() <<"\n";
  }*/

  /*adi::visualization::PointCloud pc(std::string("tcp://127.0.0.1:6000"));
  pc.loadPointCloud("/adi/test");*/

  return 0;
}