/*
 * StatusDefs.hpp
 *
 *  Created on: April 27, 2020
 *      Author: Yakun Ma
 */

#ifndef STATUSDEFS_HPP_
#define STATUSDEFS_HPP_

#include <vector>
#include <string>

namespace fvr {

//! system status data
struct SystemStatusData
{
    //! emergency stop state
    bool m_emergencyStop;

    //! external active state
    bool m_externalActive;

    //! program request state
    bool m_programRequest;

    //! program running state
    bool m_programRunning;

    //! ready pose state
    bool m_reachedReady;

    //! target pose state
    bool m_reachedTarget;

    //! control mode state
    int32_t m_ctrlMode;

    //! motion command success rate
    double m_motionCmdSuccessRate;

    //! error message
    std::string m_errorMsg;

    //! if joint limit triggered
    bool m_jntLimitTriggered;
};

//! robot status data
struct RobotStatusData
{
    //! current joint position
    std::vector<double> m_jntPos;

    //! current joint velocity
    std::vector<double> m_jntVel;

    //! current joint acceleration
    std::vector<double> m_jntAcc;

    //! current joint torque
    std::vector<double> m_jntTorque;

    //! current time stamp
    double m_timeStamp;

    //! trajectory sequence
    int32_t m_sequence;

    //! current external joint torque
    std::vector<double> m_jntExtTorque;

    //! current TCP pose
    std::vector<double> m_tcpPose;

    //! current Camera pose
    std::vector<double> m_camPose;

    //! current TCP wrench
    std::vector<double> m_tcpWrench;
};

//! plan info data
struct PlanInfoData
{
    //! current primitive name
    std::string m_ptName;

    //! current node name
    std::string m_nodeName;

    //! current node path
    std::string m_nodePath;

    //! current node path time period
    std::string m_nodePathTimePeriod;

    //! current node path number
    std::string m_nodePathNumber;

    //! assigned plan name
    std::string m_assignedPlanName;

    //! velocity scale
    double m_velocityScale;
};

} /* namespace fvr */

#endif /* STATUSDEFS_HPP_ */
