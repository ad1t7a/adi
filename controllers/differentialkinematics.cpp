#include "controllers/differentialkinematics.hpp"
#include "common/constants.hpp"

namespace adi {
namespace controllers {
// constructor
DifferentialKinematics::DifferentialKinematics(std::string path) {
  options.mLogToConsole = true;
  m_model = new RigidBodyDynamics::Model();
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(path.c_str(), m_model,
                                                   m_urdfModel, false)) {
    return;
  }
}

// destructor
DifferentialKinematics::~DifferentialKinematics() {}

// step controller
void DifferentialKinematics::step(unsigned int &bodyID, Eigen::VectorXd &jntPos,
                                  Eigen::VectorXd &jntVel,
                                  Eigen::VectorXd &cartVel,
                                  Eigen::VectorXd &cmdJntVel) {
  MatrixXd jacobianOriPos = MatrixXd::Zero(kCartPoseDofs, getDoF());
  RigidBodyDynamics::Math::Vector3d pointPosition =
      RigidBodyDynamics::Math::Vector3d::Zero();
  RigidBodyDynamics::CalcPointJacobian6D(*m_model, jntPos, bodyID,
                                         pointPosition, jacobianOriPos, true);
  try {
    // set up QP
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_LogToConsole, options.mLogToConsole);

    // initialize optimization variables
    Eigen::Matrix<GRBVar, Eigen::Dynamic, 1> qdot(getDoF());
    for (size_t i = 0; i < jacobianOriPos.cols(); i++) {
      qdot[i] = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS,
                             std::string("qdot") + std::to_string(i));
    }
    GRBVar alpha = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS, "alpha");

    // objective function (min -alpha)
    GRBQuadExpr objective = -1.0 * alpha;
    model.setObjective(objective);

    // constraint (0<=alpha<=1)
    model.addConstr(alpha >= 0, "c0");
    model.addConstr(alpha <= 1, "c1");

    // constraint (Jqdot = alpha*v)
    for (size_t row = 0; row < jacobianOriPos.rows(); row++) {
      GRBLinExpr constraintEqn;
      for (size_t col = 0; col < jacobianOriPos.cols(); col++) {
        constraintEqn += jacobianOriPos(row, col) * qdot[col];
      }
      model.addConstr(constraintEqn == alpha * cartVel[row],
                      std::string("cJqdotalphav") + std::to_string(row));
    }

    // joint position velocity and acceleration limits
    int jntIndex = 0;
    for (std::map<std::string, my_shared_ptr<urdf::Joint>>::iterator it =
             m_urdfModel->joints_.begin();
         it != m_urdfModel->joints_.end(); ++it) {
      if (it->second->type == urdf::Joint::FIXED) {
        continue;
      }
      // velocity constraints
      model.addConstr(qdot[jntIndex] >= -1.0 * it->second->limits->velocity,
                      std::string("minVel") + std::to_string(jntIndex));
      model.addConstr(qdot[jntIndex] <= it->second->limits->velocity,
                      std::string("maxVel") + std::to_string(jntIndex));

      //! canonical timestep
      const double ktimeStep = 0.0001;

      // position constraints
      model.addConstr(jntPos[jntIndex] + (ktimeStep * qdot[jntIndex]) >=
                          it->second->limits->lower,
                      std::string("minPos") + std::to_string(jntIndex));
      model.addConstr(jntPos[jntIndex] + (ktimeStep * qdot[jntIndex]) <=
                          it->second->limits->upper,
                      std::string("maxPos") + std::to_string(jntIndex));

      // acceleration constraints
      model.addConstr(((qdot[jntIndex] - jntVel[jntIndex])) >= -5.5,
                      std::string("minAcc") + std::to_string(jntIndex));
      model.addConstr(((qdot[jntIndex] - jntVel[jntIndex])) <= 5.5,
                      std::string("maxAcc") + std::to_string(jntIndex));
      jntIndex++;
    }
    model.optimize();
    for (size_t i = 0; i < jntPos.size(); i++) {
      cmdJntVel[i] = qdot[i].get(GRB_DoubleAttr_X);
    }
  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
}
} // namespace controllers
} // namespace adi