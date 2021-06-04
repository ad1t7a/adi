#include "multibody/inversekinematics/inversekinematics.hpp"

namespace adi {
namespace multibody {
// constructor
InverseKinematics::InverseKinematics(std::string path) {
  options.mLogToConsole = false;
  m_model = new RBDLMultiBody(path.c_str(), false);
}

// destructor
InverseKinematics::~InverseKinematics() { delete (m_model); }

void InverseKinematics::solve(unsigned int &bodyID, Eigen::VectorXd &jntPos,
                              Frame &cartPose) {
  // get current point frame transformation
  m_model->updateKinematics(jntPos);
  Frame currentFrame;
  m_model->getTransformationMatrix(bodyID, currentFrame);
  Frame posWorldTcp;
  posWorldTcp.m_pos << 0, 0, 0.08;
  currentFrame = currentFrame * posWorldTcp;

  // get jacobian
  Vector3d pointOnLink = Vector3d::Zero();
  size_t dof = m_model->getDoFs();

  // set up cartesian velocity to direct the robot
  Vector6d cartVel;

  // joint velocity command to update the values
  VectorXd cmdJntVel = VectorXd::Zero(dof);

  // set up QP
  GRBEnv env = GRBEnv();
  // solve for IK
  while (currentFrame.computePosDistance(cartPose) >
         options.mCartesianPositionError) {
    MatrixXd jacobian;
    m_model->getJacobian(bodyID, pointOnLink, true, jacobian);

    Vector3d posDiff = (cartPose.m_pos - currentFrame.m_pos);
    for (size_t i = 0; i < cartPose.m_pos.size(); i++) {
      posDiff[i] = (fabs(posDiff[i]) < options.mCartesianPosErrorRoundToZero)
                       ? 0.0
                       : posDiff[i];
    }
    Vector3d angleDiff;
    Frame test = cartPose * currentFrame.inverse();
    angleDiff = test.getAxisAngleRotation().axis() *
                test.getAxisAngleRotation().angle();
    for (size_t i = 0; i < angleDiff.size(); i++) {
      angleDiff[i] =
          (fabs(angleDiff[i]) < options.mCartesianOriErrorRoundToZero)
              ? 0.0
              : angleDiff[i];
    }
    cartVel << posDiff[0], posDiff[1], posDiff[2], angleDiff[0], angleDiff[1],
        angleDiff[2];
    try {
      GRBModel model = GRBModel(env);
      model.set(GRB_IntParam_LogToConsole, options.mLogToConsole);

      // initialize optimization variables
      Eigen::Matrix<GRBVar, Eigen::Dynamic, 1> qdot(dof);
      for (size_t i = 0; i < jacobian.cols(); i++) {
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
        for (size_t row = 0; row < jacobian.rows(); row++) {
          GRBLinExpr constraintEqn;
          for (size_t col = 0; col < jacobian.cols(); col++) {
            constraintEqn += jacobian(row, col) * qdot[col];
          }
          constraintEqn -= cartVel[row];
          primaryObjective += constraintEqn * constraintEqn;
        }
      }
      objective += primaryObjective;

      /*********************************************************
       *                   secondary objective function         *
       *********************************************************/
      GRBQuadExpr secondaryObjective;
      for (size_t jntIndex = 0; jntIndex < dof; jntIndex++) {
        GRBLinExpr termA = (jntPos[jntIndex] + (kTimestep * qdot[jntIndex]));
        GRBQuadExpr secondaryObjectiveTerms =
            (termA - jntPos[jntIndex]) * (termA - jntPos[jntIndex]);
        secondaryObjective += 100.0 * secondaryObjectiveTerms;
      }
      objective += secondaryObjective;
      model.setObjective(objective);

      /*********************************************************
       *                   constraint equations                 *
       *********************************************************/
      if (!options.mObstacleAvoidance) {
        // constraint (0<=alpha<=1)
        model.addConstr(alpha >= 0, "c0");
        model.addConstr(alpha <= 1, "c1");

        // constraint (Jqdot = alpha*v)
        for (size_t row = 0; row < jacobian.rows(); row++) {
          GRBLinExpr constraintEqn;
          for (size_t col = 0; col < jacobian.cols(); col++) {
            constraintEqn += jacobian(row, col) * qdot[col];
          }
          model.addConstr(constraintEqn == alpha * cartVel[row],
                          std::string("cJqdotalphav") + std::to_string(row));
        }
      }

      // joint position velocity and acceleration limits
      int jntIndex = 0;
      for (std::map<std::string, my_shared_ptr<urdf::Joint>>::iterator it =
               m_model->getURDFModel()->joints_.begin();
           it != m_model->getURDFModel()->joints_.end(); ++it) {
        if (it->second->type == urdf::Joint::FIXED) {
          continue;
        }
        // position constraints
        /*model.addConstr(jntPos[jntIndex] + (kTimestep * qdot[jntIndex]) >=
                            it->second->limits->lower,
                        std::string("minPos") + std::to_string(jntIndex));
        model.addConstr(jntPos[jntIndex] + (kTimestep * qdot[jntIndex]) <=
                            it->second->limits->upper,
                        std::string("maxPos") + std::to_string(jntIndex));*/
        jntIndex++;
      }

      model.optimize();
      if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
        spdlog::error("Unable to solve IK: error {:d}",
                      model.get(GRB_IntAttr_Status));
        m_model->updateKinematics(jntPos);
        m_model->getTransformationMatrix(bodyID, currentFrame);
        currentFrame = currentFrame * posWorldTcp;
        return;
      }

      for (size_t i = 0; i < jntPos.size(); i++) {
        cmdJntVel[i] = qdot[i].get(GRB_DoubleAttr_X);
      }
      jntPos += cmdJntVel;
      // get current point frame transformation
      m_model->updateKinematics(jntPos);
      m_model->getTransformationMatrix(bodyID, currentFrame);
      currentFrame = currentFrame * posWorldTcp;
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
}
} // namespace multibody
} // namespace adi