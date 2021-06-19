// Copyright (C) Flexiv Ltd. 2020. All Rights Reserved.
/*
 * FvrRobotClient.hpp
 *
 *  Created on: Jun 14, 2020
 *      Author: Yizhou
 */

#ifndef FVRROBOTCLIENT_HPP_
#define FVRROBOTCLIENT_HPP_

#include <vector>
#include <memory>
#include <inttypes.h>

#include "StatusDefs.hpp"
#include "EthernetCtrlMode.hpp"

namespace fvr {

//! ============================================================================
//!               class for Robot TCP Client
//! ============================================================================
class RobotClient;

class FvrRobotClient
{
public:
    FvrRobotClient();
    virtual ~FvrRobotClient();

    //! initialize robot client
    bool init(const std::string& serverAddr, const std::string& localAddr,
        unsigned int dofs);

    //! if client is connected with server
    bool connected(void) const;

    //! if heartbeat is timeout
    bool timeout(void) const;

    //! connect with robot server
    bool connect(void);

    //! disconnect with robot server
    void disconnect(void);

    //! stop the robot while moving
    //! control mode will change to Idle when robot stopped
    bool stop(void);

    //-----------------------------SYSTEM CMD---------------------------------
    //! execute plan by index
    bool executePlanByIndex(int32_t index);

    //! execute plan by name
    bool executePlanByName(const std::string& name);

    //! execute primitive
    bool executePrimitive(const std::string& ptCmd);

    //! stop plan while it is running
    bool stopPlanExecution(void);

    //! set robot control mode
    bool setControlMode(ExtCtrlMode ctrlMode);

    //! switch tcp index
    bool switchTcp(int32_t index);

    //! get name list of all the available plans
    bool getPlanNameList(std::vector<std::string>* planNameList);

    //! get current running plan info
    bool getPlanInfo(PlanInfoData* planInfo);

    //! if robot is in fault state
    bool isFault(void);

    //! clear robot minor fault
    bool clearFault(void);

    //! set recovery mode
    bool setRecoveryMode(RecoveryMode mode);

    //------------------------------MOTION CMD-------------------------------
    //! send joint pvat command
    bool sendJointPVAT(const std::vector<std::vector<double> >& position,
        const std::vector<std::vector<double> >& velocity,
        const std::vector<std::vector<double> >& acceleration,
        std::vector<double> time);

    //! send joint torque command
    bool sendJointTorque(const std::vector<double>& torque, int32_t index,
        bool gravityCompensation = true);

    //! stream joint pvat command
    bool streamJointPVAT(const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& acceleration, int32_t index);

    //! send tcp pose command
    bool sendTcpPose(const std::vector<double>& tcpPose,
        const std::vector<double>& maxWrench, double time, int32_t index);

    //! send online move command
    bool sendOnlinePose(const std::vector<double>& tcpPose,
        const std::vector<double>& tcpTwist, double maxV, double maxA,
        double maxW, double maxdW, int32_t index);

    //! send move line command
    bool sendMoveLineWaypoints(
        const std::vector<std::vector<double> >& waypoints,
        const std::vector<double>& maxV, const std::vector<double>& maxA,
        const std::vector<double>& maxW, const std::vector<double>& maxdW,
        const std::vector<unsigned int>& level);

    //! send move circle command
    bool sendMoveCircleWaypoints(
        const std::vector<std::vector<double> >& waypoints, double maxV,
        double maxW);

    //! send joint jogging command
    bool sendJointJogPosition(unsigned int jointIndex, double position);

    //-------------------------------IO CMD-----------------------------------
    //! write IO with index
    bool writeDigitalOutput(int32_t portNumber, bool value);

    //! read IO with index
    bool readDigitalInput(int32_t portNumber, bool* value);

    //----------------------------GET ROBOT STATUS----------------------------
    bool readRobotStatus(RobotStatusData* robotStatus);

    //----------------------------GET SYSTEM STATUS----------------------------
    //! read system status
    bool readSystemStatus(SystemStatusData* systemStatus);

    //! if target has reached
    bool targetReached(void);

    //-------------------------GET ROBOT CONTROL MODE--------------------------
    //! get current robot control mode
    ExtCtrlMode getCtrlMode(void);

private:
    std::unique_ptr<RobotClient> m_client;
};

} /* namespace fvr */

#endif /* FVRROBOTCLIENT_HPP_ */
