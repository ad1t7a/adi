/*
 * EthernetCtrlMode.hpp
 *
 *  Created on: Sep 30, 2020
 *      Author: Yakun Ma
 */

#ifndef ETHERNETCTRLMODE_HPP_
#define ETHERNETCTRLMODE_HPP_

namespace fvr {

//! control mode definition for ethernet interface
enum ExtCtrlMode
{
    CTRL_MODE_UNKNOWN = -1,
    CTRL_MODE_IDLE,

    //! joint space mode
    CTRL_MODE_JOINT_PVAT_DOWNLOAD,
    CTRL_MODE_JOINT_PVAT_STREAMING,
    CTRL_MODE_JOINT_TORQUE,

    //! cartesian space mode
    CTRL_MODE_CARTESIAN_POSE,
    CTRL_MODE_ONLINE_MOVE,
    CTRL_MODE_MOVE_LINE,
    CTRL_MODE_MOVE_CIRCLE,

    //! general plan execution mode
    CTRL_MODE_PLAN_EXECUTION,

    //! general primitive execution mode
    CTRL_MODE_PRIMITIVE_EXECUTION,

    CTRL_MODE_FIRST = CTRL_MODE_IDLE,
    CTRL_MODE_LAST = CTRL_MODE_PRIMITIVE_EXECUTION,
    CTRL_MODE_NUM = CTRL_MODE_LAST - CTRL_MODE_FIRST + 1
};

static const std::string ExtCtrlModeTypeName[CTRL_MODE_NUM]
    = {"Idle", "JointPVATDownload", "JointPVATStreaming", "JointTorque",
        "CartesianPose", "OnlineMove", "MoveLine", "MoveCircle",
        "PlanExecution", "PrimitiveExecution"};

enum RecoveryMode
{
    //! robot will hold current position
    RECOVERY_MODE_HOLD,

    //! robot could receive joint position command
    RECOVERY_MODE_JOINT_JOG,

    //! robot could be manually driven (with mobar enabled)
    RECOVERY_MODE_FREE_DRIVE,
};

} /* namespace fvr */

#endif /* ETHERNETCTRLMODE_HPP_ */
