#include "controllers/cartesian/differentialkinematics.hpp"
#include "common/constants.hpp"

namespace adi {
namespace controllers {
// constructor
DifferentialKinematics::DifferentialKinematics(std::string path) {
  options.mLogToConsole = false;
  m_model = new RigidBodyDynamics::Model();
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(path.c_str(), m_model,
                                                   m_urdfModel, false)) {
    return;
  }
  mJacobianOriPos = MatrixXd::Zero(kCartPoseDofs, getDoF());
  
}

// destructor
DifferentialKinematics::~DifferentialKinematics() {}

// step controller
void DifferentialKinematics::step(unsigned int &bodyID, Eigen::VectorXd &jntPos,
                                  Eigen::VectorXd &jntVel,
                                  Eigen::VectorXd &cartVel,
                                  Eigen::VectorXd &cmdJntVel) {
  RigidBodyDynamics::Math::Vector3d pointPosition =
      RigidBodyDynamics::Math::Vector3d::Zero();
  RigidBodyDynamics::CalcPointJacobian6D(*m_model, jntPos, bodyID,
                                         pointPosition, mJacobianOriPos, true);
  try {
    // set up QP
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_LogToConsole, options.mLogToConsole);

    // initialize optimization variables
    Eigen::Matrix<GRBVar, Eigen::Dynamic, 1> qdot(getDoF());
    for (size_t i = 0; i < mJacobianOriPos.cols(); i++) {
      qdot[i] = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS,
                             std::string("qdot") + std::to_string(i));
    }
    GRBVar alpha = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS, "alpha");

    // objective function
    GRBQuadExpr objective;

    // primary objective function (min -alpha)
    GRBQuadExpr primaryObjective = -1.0 * alpha;
    objective += primaryObjective;
    // secondary objective
    if (mJacobianOriPos.rows() < mJacobianOriPos.cols()) {
      for (size_t jntIndex = 0; jntIndex < getDoF(); jntIndex++) {
        GRBLinExpr termA = (jntPos[jntIndex] + (kTimestep * qdot[jntIndex]));
        GRBQuadExpr secondaryObjectiveTerms =
            (termA - jntPos[jntIndex]) * (termA - jntPos[jntIndex]);
        objective += secondaryObjectiveTerms;
      }
    }

    model.setObjective(objective);

    // constraint (0<=alpha<=1)
    model.addConstr(alpha >= 0, "c0");
    model.addConstr(alpha <= 1, "c1");

    // constraint (Jqdot = alpha*v)
    for (size_t row = 0; row < mJacobianOriPos.rows(); row++) {
      GRBLinExpr constraintEqn;
      for (size_t col = 0; col < mJacobianOriPos.cols(); col++) {
        constraintEqn += mJacobianOriPos(row, col) * qdot[col];
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

      // position constraints
      model.addConstr(jntPos[jntIndex] + (kTimestep * qdot[jntIndex]) >=
                          it->second->limits->lower,
                      std::string("minPos") + std::to_string(jntIndex));
      model.addConstr(jntPos[jntIndex] + (kTimestep * qdot[jntIndex]) <=
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
    for (size_t i = 0; i < jntPos.size(); i++) {
      cmdJntVel[i] = 0.0;
    }
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
    for (size_t i = 0; i < jntPos.size(); i++) {
      cmdJntVel[i] = 0.0;
    }
  }
}
} // namespace controllers
} // namespace adi