#include "controllers/cartesian/differentialkinematics.hpp"
#include "common/constants.hpp"
#include "common/frame.hpp"

namespace adi {
namespace controllers {
// constructor
DifferentialKinematics::DifferentialKinematics(std::string path) {
  options.mLogToConsole = false;
  m_dynamics = new multibody::RBDLMultiBody(path.c_str(), false);
  mJacobianOriPos = MatrixXd::Zero(kCartPoseDofs, m_dynamics->getDoFs());
}

// destructor
DifferentialKinematics::~DifferentialKinematics() {}

// step controller
void DifferentialKinematics::step(unsigned int bodyID, Eigen::VectorXd &jntPos,
                                  Eigen::VectorXd &jntVel, Vector6d &cartVel,
                                  Eigen::VectorXd &cmdJntVel) {
  RigidBodyDynamics::Math::Vector3d pointPosition =
      RigidBodyDynamics::Math::Vector3d::Zero();

  m_dynamics->updateKinematics(jntPos, jntVel);
  m_dynamics->getJacobian(bodyID, pointPosition, true, mJacobianOriPos);

  // Inertia matrix
  MatrixXd inertiaMatrix;
  inertiaMatrix = MatrixXd::Zero(m_dynamics->getDoFs(), m_dynamics->getDoFs());
  VectorXd currentTau;
  currentTau = VectorXd::Zero(m_dynamics->getDoFs());
  m_dynamics->getInertiaMatrix(jntPos, inertiaMatrix);
  m_dynamics->getNonLinearTerms(jntPos, jntVel, currentTau);

  try {
    // set up QP
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_LogToConsole, options.mLogToConsole);

    // initialize optimization variables
    Eigen::Matrix<GRBVar, Eigen::Dynamic, 1> qdot(m_dynamics->getDoFs());
    for (size_t i = 0; i < mJacobianOriPos.cols(); i++) {
      qdot[i] = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS,
                             std::string("qdot") + std::to_string(i));
    }
    GRBVar alpha = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS, "alpha");

    GRBQuadExpr objective;

    /*********************************************************
     *                   primary objective function           *
     *********************************************************/
    GRBQuadExpr primaryObjective;
    if (!options.mObstacleAvoidance) {
      // primary objective function (min -alpha)
      primaryObjective = -1.0 * alpha;
    } else {
      // constraint (Jqdot - v)^2
      for (size_t row = 0; row < mJacobianOriPos.rows(); row++) {
        GRBLinExpr constraintEqn;
        for (size_t col = 0; col < mJacobianOriPos.cols(); col++) {
          constraintEqn += mJacobianOriPos(row, col) * qdot[col];
        }
        constraintEqn -= cartVel[row];
        primaryObjective += constraintEqn * constraintEqn;
      }
    }
    objective += primaryObjective;

    /*********************************************************
     *                   secondary objective function         *
     *********************************************************/
    for (size_t jntIndex = 0; jntIndex < m_dynamics->getDoFs(); jntIndex++) {
      GRBLinExpr termA = (jntPos[jntIndex] + (kTimestep * qdot[jntIndex]));
      GRBQuadExpr secondaryObjectiveTerms =
          (termA - jntPos[jntIndex]) * (termA - jntPos[jntIndex]);
      objective += secondaryObjectiveTerms;
    }
      model.setObjective(objective);

      /*********************************************************
       *                   constraint equations                 *
       *********************************************************/
      if (!options.mObstacleAvoidance) {
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
      }

    // joint position velocity and acceleration limits
    int jntIndex = 0;
    for (std::map<std::string, my_shared_ptr<urdf::Joint>>::iterator it =
             m_dynamics->getURDFModel()->joints_.begin();
         it != m_dynamics->getURDFModel()->joints_.end(); ++it) {
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

      // torque constraints
      // TODO: Add non-linear constraints
      GRBQuadExpr trqConstr;
      for (int i = 0; i < inertiaMatrix.cols(); i++) {
        trqConstr +=
            inertiaMatrix(jntIndex, i) * ((qdot[i] - jntVel[i]) / kTimestep);
      }
      model.addConstr(trqConstr <= it->second->limits->effort,
                      std::string("maxTrq") + std::to_string(jntIndex));
      model.addConstr(trqConstr >= -1.0 * it->second->limits->effort,
                      std::string("minTrq") + std::to_string(jntIndex));
      jntIndex++;
    }
    if (options.mObstacleAvoidance) {
      // add obstacle constraints
      /*int bodyId = 6;
      Frame curFrame;
      const RigidBodyDynamics::Math::SpatialTransform &st =
      m_model->X_base[bodyId]; Affine3d affine = Translation(st.r) *
      st.E.transpose(); curFrame.m_pos = affine.translation(); curFrame.m_rot =
      affine.rotation(); GRBLinExpr val = curFrame.m_pos[2] + (cartVel[5] *
      kTimestep); model.addConstr(val >= 0.3, std::string("posframe"));
      //model.addConstr(curFrame.m_pos[2] + (cartVel[5] * kTimestep) <= 0.6,
      std::string("posframe2")); std::cout << curFrame.m_pos.transpose()
      <<"\n";*/
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