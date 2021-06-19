// Copyright (C) Flexiv Ltd. 2020. All Rights Reserved.
//=============================================================================
/*!
 * \brief Robot client test class
 *
 *  Created on: June 23, 2020
 *      Author: Yakun Ma
 */
//=============================================================================
#ifndef CLIENTTESTFUNCTION_HPP_
#define CLIENTTESTFUNCTION_HPP_

#include <sched.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <map>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>

#include "FvrRobotClient.hpp"
#include <Eigen/Dense>
#define TIMER_ABSTIME 1
#define _FVR_INFO(...)                                                         \
    do {                                                                       \
        printf(__VA_ARGS__);                                                   \
        puts("");                                                              \
    } while (0);

#define _FVR_WARNING _FVR_INFO
#define _FVR_ERROR _FVR_INFO

#define FVR_PI M_PI
#define DegreeToRadian(x) x * 3.1415 / 180.0
#define Sign(val) (0 < val) - (val < 0)
#define kControlLoopInterval 0.001
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_SEC (1000000000)
using namespace fvr;
constexpr double k_controlLoopInterval = 0.001;
constexpr double k_secondToMillisecond = 1000.0;
class ClientTestFunction
{

public:
    ClientTestFunction(std::shared_ptr<FvrRobotClient> robotClient)
    {
        // ---------------1. get and print robot status--------------
        auto robotStatus = [this, robotClient]() {
            RobotStatusData robotStatus;
            while (true) {
                if (true != robotClient->readRobotStatus(&robotStatus)) {
                    return false;
                }
                printVectorInfo(robotStatus.m_jntPos, "position:", true);
                printVectorInfo(robotStatus.m_jntVel, "velocity:", true);
                printVectorInfo(robotStatus.m_jntAcc, "acceleration:", true);
                printVectorInfo(robotStatus.m_jntTorque, "torque:");
                printVectorInfo(robotStatus.m_tcpPose, "tcp pose:");
                printVectorInfo(robotStatus.m_tcpWrench, "tcp wrench:");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        };

        // ---------------2. get and print system status--------------
        auto sysStatus = [this, robotClient]() {
            SystemStatusData systemStatus;
            while (true) {
                if (true != robotClient->readSystemStatus(&systemStatus)) {
                    return false;
                }
                _FVR_INFO("emergency:%d", systemStatus.m_emergencyStop);
                _FVR_INFO("externalActive:%d", systemStatus.m_externalActive);
                _FVR_INFO("programRequest:%d", systemStatus.m_programRequest);
                _FVR_INFO("programRunning:%d", systemStatus.m_programRunning);
                _FVR_INFO("reachedReady:%d\n", systemStatus.m_reachedReady);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        };

        // ------------------3. read digital input port------------------
        auto readIO = [this, robotClient]() {
            unsigned int portNumber;
            _FVR_INFO("please enter input port number:");
            std::cin >> portNumber;
            if (portNumber > 15) {
                _FVR_WARNING("exceed maximum port number!");
                return false;
            }
            bool value;
            if (true != robotClient->readDigitalInput(portNumber, &value)) {
                return false;
            }
            _FVR_INFO("portNumber:%d, value:%d", portNumber, value);

            return true;
        };

        // ------------------4. write digital output port------------------
        auto writeIO = [this, robotClient]() {
            unsigned int portNumber;
            unsigned int value;
            _FVR_INFO("please enter output port number:");
            std::cin >> portNumber;
            _FVR_INFO("please enter output value:");
            std::cin >> value;
            if (true
                != robotClient->writeDigitalOutput(
                       portNumber, static_cast<bool>(value))) {
                return false;
            }

            return true;
        };

        // ------------------5. manipulation plan execution----------------
        auto planExecution = [this, robotClient]() {
            if (true != robotClient->setControlMode(CTRL_MODE_PLAN_EXECUTION)) {
                return false;
            }

            // wait for robot to enter plan execution mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robotClient->getCtrlMode() != CTRL_MODE_PLAN_EXECUTION);

            std::vector<std::string> planList;
            if (true != robotClient->getPlanNameList(&planList)) {
                return false;
            }

            _FVR_INFO(
                "===================== Plan name list =====================");
            for (unsigned int i = 0; i < planList.size(); i++) {
                _FVR_INFO("[%u] %s", i, planList[i].c_str());
            }

            unsigned int index;
            _FVR_INFO("please enter plan index:");
            std::cin >> index;
            _FVR_INFO("entered plan index: %u", index);
            if (true != robotClient->executePlanByIndex(index)) {
                return false;
            }

            SystemStatusData systemStatus;
            do {
                // wait until execution begin
                if (true != robotClient->readSystemStatus(&systemStatus)) {
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (systemStatus.m_programRunning == false);

            do {
                // wait until execution finished
                if (true != robotClient->readSystemStatus(&systemStatus)) {
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (systemStatus.m_programRunning == true);

            _FVR_INFO("plan execution finished");

            return true;
        };

        // ------------------6. master slave control--------------------
        auto masterSlave = [this, robotClient]() {
            robotClient->setControlMode(CTRL_MODE_CARTESIAN_POSE);
            robotClient->switchTcp(0);
            std::vector<double> pose = {0.684, -0.110, 0.269, 0, 0, 1, 0};
            std::vector<double> wrench = {10.0, 10.0, 10.0, 20.0, 20.0, 20.0};
            double time = 0.0;
            unsigned int index = 0;

            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);

            // send command every millisecond
            while (true) {

                pose[0] = 0.684 + 0.02 * sin(FVR_PI * time);
                if (robotClient->sendTcpPose(
                        pose, wrench, time += 0.001, index++)
                    != true) {
                    break;
                }
                _Clock_NextInterval_ms(ts,
                    (uint32_t)(k_controlLoopInterval * k_secondToMillisecond));
                std::this_thread::sleep_for(std::chrono::nanoseconds((uint32_t)(k_controlLoopInterval * k_secondToMillisecond)));
            }
            return false;
        };

        // --------------------7. move line mode--------------------------
        auto moveLine = [this, robotClient]() {
            // set mode to CTRL_MODE_MOVE_LINE
            robotClient->setControlMode(CTRL_MODE_MOVE_LINE);

            // wait for robot to enter move line mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robotClient->getCtrlMode() != CTRL_MODE_MOVE_LINE);

            // send the waypoints to robot
            std::vector<std::vector<double>> waypoints;
            std::vector<double> pose1 = {0.687, 0.110, 0.203, 0.1830127,
                0.6830127, 0.6830127, -0.1830127};
            std::vector<double> pose2 = {
                0.687, 0.110, 0.143, 0.0005633, 0.707388, 0.706825, 0.0005629};
            std::vector<double> pose3
                = {0.687, -0.110, 0.143, 0.0, 0.0, 1.0, 0.0};
            std::vector<double> pose4 = {
                0.687, -0.110, 0.083, 0.0005633, 0.707388, 0.706825, 0.0005629};

            waypoints.push_back(pose1);
            waypoints.push_back(pose2);
            waypoints.push_back(pose3);
            waypoints.push_back(pose4);

            std::vector<double> maxV = {1.0, 0.01, 1.0, 0.01};
            std::vector<double> maxA = {6.0, 6.0, 6.0, 6.0};
            std::vector<double> maxW = {3.0, 3.0, 3.0, 3.0};
            std::vector<double> maxdW = {3.0, 3.0, 3.0, 3.0};
            std::vector<unsigned int> level = {0, 6, 0, 0};

            robotClient->sendMoveLineWaypoints(
                waypoints, maxV, maxA, maxW, maxdW, level);

            // check if target has reached
            while (robotClient->targetReached() == false) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            _FVR_INFO("move line target reached");

            robotClient->setControlMode(CTRL_MODE_IDLE);

            // wait for robot to enter idle mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robotClient->getCtrlMode() != CTRL_MODE_IDLE);

            _FVR_INFO("robot set to idle mode");

            return true;
        };

        // ----------------------8. online move mode----------------------
        auto onlineMove = [this, robotClient]() {
            // set mode to CTRL_MODE_ONLINE_MOVE
            robotClient->setControlMode(CTRL_MODE_ONLINE_MOVE);

            // wait for robot to enter online move mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robotClient->getCtrlMode() != CTRL_MODE_ONLINE_MOVE);

            RobotStatusData robotStatus;
            robotClient->readRobotStatus(&robotStatus);
            std::vector<double> initPose = robotStatus.m_tcpPose;

            double maxV = 1.0;
            double maxA = 0.5;
            double maxW = 1.5;
            double maxdW = 2.0;

            std::vector<double> targetPose(7);
            std::vector<double> targetTwist(6, 0.0);
            int32_t index = 0;

            // pose generated online
            targetPose[0] = initPose[0];
            targetPose[1] = initPose[1];
            targetPose[2] = initPose[2] + 0.02;

            targetPose[3] = initPose[3];
            targetPose[4] = initPose[4];
            targetPose[5] = initPose[5];
            targetPose[6] = initPose[6];

            robotClient->sendOnlinePose(
                targetPose, targetTwist, maxV, maxA, maxW, maxdW, ++index);

            return false;
        };

        // ---------------------9. joint pvat command---------------------
        auto jointPvat = [this, robotClient]() {
            // set mode to CTRL_MODE_JOINT_PVAT_DOWNLOAD
            robotClient->setControlMode(CTRL_MODE_JOINT_PVAT_DOWNLOAD);

            // wait for robot to enter joint PVAT download mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (
                robotClient->getCtrlMode() != CTRL_MODE_JOINT_PVAT_DOWNLOAD);

            // send the waypoints to robot
            std::vector<std::vector<double>> position;
            std::vector<double> pos1
                = {25.0, -30.0, 0.0, -90.0, 0.0, 40.0, 0.0};
            std::vector<double> pos2
                = {75.0, -10.0, 0.0, -90.0, 0.0, 40.0, 0.0};
            std::vector<double> pos3
                = {50.0, -40.0, 0.0, -90.0, 20.0, 40.0, 0.0};
            std::vector<double> pos4
                = {0.0, -40.0, 0.0, -90.0, 10.0, 40.0, 5.0};
            position.push_back(pos1);
            position.push_back(pos2);
            position.push_back(pos3);
            position.push_back(pos4);

            std::vector<std::vector<double>> velocity;
            std::vector<double> vel1 = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> vel2 = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> vel3 = {-5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> vel4 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            velocity.push_back(vel1);
            velocity.push_back(vel2);
            velocity.push_back(vel3);
            velocity.push_back(vel4);

            std::vector<std::vector<double>> acceleration;
            std::vector<double> acc1 = {3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> acc2 = {6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> acc3 = {-2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> acc4 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            acceleration.push_back(acc1);
            acceleration.push_back(acc2);
            acceleration.push_back(acc3);
            acceleration.push_back(acc4);

            std::vector<double> timeStamp = {3.0, 5.0, 8.0, 10.0};

            robotClient->sendJointPVAT(
                position, velocity, acceleration, timeStamp);

            _FVR_INFO("press 's' to stop the robot");

            std::string cmdStr;
            while (std::cin >> cmdStr) {

                // receive \n from keyboard
                getchar();

                if (cmdStr == "s") {
                    robotClient->stop();
                    break;
                }
            }

            _FVR_INFO("move joint pvat target reached");

            return true;
        };

        // ------------------10. joint position streaming----------------
        auto jointPosStreaming = [this, robotClient]() {
            // trajectory parameters definition
            unsigned int robotDof = 7;
            double trajectoryStepSize = 0.001;
            double maxVel = DegreeToRadian(4.0);
            double posRange = DegreeToRadian(10.0);
            double trajDuration = posRange / maxVel;
            double timeCounts = 0.0;
            double maxJntPosErr = DegreeToRadian(7.0);

            // set mode to CTRL_MODE_JNTPOS_STREAMING
            robotClient->setControlMode(CTRL_MODE_JOINT_PVAT_STREAMING);

            // wait for robot to enter joint joint pos streaming mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (
                robotClient->getCtrlMode() != CTRL_MODE_JOINT_PVAT_STREAMING);

            // read current joint pos
            RobotStatusData robotStatus;
            robotClient->readRobotStatus(&robotStatus);
            Eigen::VectorXd startJntPos
                = Eigen::VectorXd::Zero(robotStatus.m_jntPos.size());
            for (unsigned int i = 0; i < startJntPos.size(); i++) {
                startJntPos(i) = robotStatus.m_jntPos[i];
            }

            Eigen::VectorXd targetJntPos = startJntPos;
            Eigen::VectorXd targetJntVel = Eigen::VectorXd::Zero(robotDof);
            Eigen::VectorXd targetJntAcc = Eigen::VectorXd::Zero(robotDof);
            Eigen::VectorXd curJntPos = startJntPos;
            Eigen::VectorXd curJntVel = Eigen::VectorXd::Zero(robotDof);
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            while (true) {
                // get current joint pos
                robotClient->readRobotStatus(&robotStatus);
                for (unsigned int i = 0; i < robotDof; i++) {
                    curJntPos(i) = robotStatus.m_jntPos[i];
                    curJntVel(i) = robotStatus.m_jntVel[i];
                }

                // check joint pos error and update target joint pos
                if ((targetJntPos - curJntPos).norm() < maxJntPosErr) {
                    timeCounts += trajectoryStepSize;
                    if (timeCounts >= trajDuration) {
                        timeCounts = trajDuration;
                        targetJntVel = Eigen::VectorXd::Zero(robotDof);
                    } else {
                        targetJntVel
                            = Eigen::VectorXd::Constant(robotDof, maxVel);
                    }
                    targetJntPos
                        = targetJntPos + trajectoryStepSize * targetJntVel;
                }
                std::vector<double> cmdJntPos;
                std::vector<double> cmdJntVel;
                std::vector<double> cmdJntAcc;
                for (unsigned int i = 0; i < targetJntPos.size(); i++) {
                    cmdJntPos.push_back(targetJntPos(i));
                    cmdJntVel.push_back(targetJntVel(i));
                    cmdJntAcc.push_back(targetJntAcc(i));
                }

                // send target joint position ,velocity and acc;
                robotClient->streamJointPVAT(
                    cmdJntPos, cmdJntVel, cmdJntVel, 1);
                _Clock_NextInterval_ms(ts,
                    (uint32_t)(k_controlLoopInterval * k_secondToMillisecond));
                std::this_thread::sleep_for(std::chrono::nanoseconds((uint32_t)(k_controlLoopInterval * k_secondToMillisecond)));
            }
            return true;
        };

        // ---------------------11. joint torque streaming ---------------------
        auto jointTrqStreaming = [this, robotClient]() {
            // trajectory parameters definition
            unsigned int robotDof = 7;
            double trajectoryStepSize = 0.001;
            double maxVel = DegreeToRadian(4.0);
            double posRange = DegreeToRadian(10.0);
            double trajDuration = posRange / maxVel;
            double maxJntPosErr = DegreeToRadian(7.0);
            double timeCounts = 0.0;

            // control parameters definition
            Eigen::VectorXd maxJntTrq = Eigen::VectorXd::Zero(robotDof);
            Eigen::VectorXd Kp = Eigen::VectorXd::Zero(robotDof);
            Eigen::VectorXd Kv = Eigen::VectorXd::Zero(robotDof);
            maxJntTrq << 10, 10, 10, 10, 5, 5, 2;
            Kp << 1500, 1500, 1500, 1500, 800, 600, 200;
            Kv << 20, 20, 20, 20, 15, 15, 2;

            // set mode to CTRL_MODE_JOINT_TORQUE
            robotClient->setControlMode(CTRL_MODE_JOINT_TORQUE);

            // wait for robot to enter joint joint pos streaming mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robotClient->getCtrlMode() != CTRL_MODE_JOINT_TORQUE);

            // read current joint pos and compute end joint pos
            RobotStatusData robotStatus;
            robotClient->readRobotStatus(&robotStatus);
            Eigen::VectorXd startJntPos = Eigen::VectorXd::Zero(robotDof);
            for (unsigned int i = 0; i < startJntPos.size(); i++) {
                startJntPos(i) = robotStatus.m_jntPos[i];
            }

            Eigen::VectorXd targetJntPos = startJntPos;
            Eigen::VectorXd targetJntVel = Eigen::VectorXd::Zero(robotDof);
            Eigen::VectorXd targetJntAcc = Eigen::VectorXd::Zero(robotDof);
            Eigen::VectorXd curJntPos = startJntPos;
            Eigen::VectorXd curJntVel = Eigen::VectorXd::Zero(robotDof);
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            while (true) {
                // get current joint pos
                robotClient->readRobotStatus(&robotStatus);
                for (unsigned int i = 0; i < robotDof; i++) {
                    curJntPos(i) = robotStatus.m_jntPos[i];
                    curJntVel(i) = robotStatus.m_jntVel[i];
                }

                // check joint pos error and update target joint pos
                if ((targetJntPos - curJntPos).norm() < maxJntPosErr) {
                    timeCounts += k_controlLoopInterval;
                    if (timeCounts >= trajDuration) {
                        timeCounts = trajDuration;
                        targetJntVel = Eigen::VectorXd::Zero(robotDof);
                    } else {
                        targetJntVel
                            = Eigen::VectorXd::Constant(robotDof, maxVel);
                    }
                    targetJntPos
                        = targetJntPos + trajectoryStepSize * targetJntVel;
                }

                //! compute control torque
                Eigen::VectorXd posError = targetJntPos - curJntPos;
                Eigen::VectorXd velError = targetJntVel - curJntVel;
                Eigen::VectorXd ctrlTorque = Kp.array() * posError.array()
                                             + Kv.array() * velError.array();
                std::vector<double> cmdJntTorq;
                for (unsigned int i = 0; i < ctrlTorque.size(); i++) {
                    if (fabs(ctrlTorque(i)) > maxJntTrq[i]) {
                        ctrlTorque(i) = Sign(ctrlTorque(i)) * maxJntTrq[i];
                    }
                    cmdJntTorq.push_back(ctrlTorque(i));
                }

                // send joint torque
                robotClient->sendJointTorque(cmdJntTorq, 1);
                _Clock_NextInterval_ms(ts,
                    (uint32_t)(k_controlLoopInterval * k_secondToMillisecond));
                std::this_thread::sleep_for(std::chrono::nanoseconds((uint32_t)(k_controlLoopInterval * k_secondToMillisecond)));
            }
            return true;
        };

        // ---------------------12. get plan name list ---------------------
        auto getPlanNameList = [this, robotClient]() {
            std::vector<std::string> planNameList;
            robotClient->getPlanNameList(&planNameList);

            for (unsigned int i = 0; i < planNameList.size(); i++) {
                _FVR_INFO("[%u] %s", i, planNameList[i].c_str());
            }

            return true;
        };

        // -------------------13. Idle mode----------------
        auto idleMode = [this, robotClient]() {
            robotClient->setControlMode(CTRL_MODE_IDLE);

            // wait for robot to enter idle mode
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robotClient->getCtrlMode() != CTRL_MODE_IDLE);

            _FVR_INFO("robot set to idle mode");

            return true;
        };

        // -------------------14. Joint jog in recovery mode----------------
        auto jointJog = [this, robotClient]() {
            robotClient->setRecoveryMode(RECOVERY_MODE_JOINT_JOG);

            robotClient->sendJointJogPosition(6, 0.0);

            _FVR_INFO("send joint jog");

            return true;
        };

        // -------------------15. Free drive in recovery mode----------------
        auto freeDrive = [this, robotClient]() {
            robotClient->setRecoveryMode(RECOVERY_MODE_FREE_DRIVE);

            _FVR_INFO("enable free drive");

            return true;
        };

        // ------------------16. primitive execution----------------
        auto ptExecution = [this, robotClient]() {
            robotClient->setControlMode(CTRL_MODE_PRIMITIVE_EXECUTION);

            do {
                _FVR_INFO("waiting");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (
                robotClient->getCtrlMode() != CTRL_MODE_PRIMITIVE_EXECUTION);

            std::string ptCmd;
            _FVR_INFO("Please enter primitive command:\n");
            _FVR_INFO(
                "A valid primitive command follows the following format:");
            _FVR_INFO("ptName(inputParam1=xxx, inputParam2=xxx, ...)\n");
            std::getline(std::cin, ptCmd);

            robotClient->executePrimitive(ptCmd);

            return true;
        };

        registerTestCase(robotStatus, "Get robot status");
        registerTestCase(sysStatus, "Get system status");
        registerTestCase(readIO, "Read Digital Input");
        registerTestCase(writeIO, "Write Digital Output");
        registerTestCase(planExecution, "Execute plan by index or name");
        registerTestCase(masterSlave, "Master slave control");
        registerTestCase(moveLine, "Send move line command");
        registerTestCase(onlineMove, "Online move control");
        registerTestCase(jointPvat, "Send joint PVAT command");
        registerTestCase(
            jointPosStreaming, "Send joint Position Streaming command");
        registerTestCase(
            jointTrqStreaming, "Send joint torque Streaming command");
        registerTestCase(getPlanNameList, "Get plan name list");
        registerTestCase(idleMode, "Set robot to Idle mode");
        registerTestCase(moveLine, "Move line");
        registerTestCase(jointJog, "Joint jog in recovery mode");
        registerTestCase(freeDrive, "Free drive in recovery mode");
        registerTestCase(
            ptExecution, "Execute primitive by name and parameters");
    }
    virtual ~ClientTestFunction() {}

    void _Clock_NextInterval_ms(struct timespec& ts, uint32_t ms)
    {
        ts.tv_nsec += ms * NSEC_PER_MSEC;
        if (ts.tv_nsec >= NSEC_PER_SEC) {
            ts.tv_sec++;
            ts.tv_nsec -= NSEC_PER_SEC;
        }
    }

    bool operator()(int testIndex)
    {
        if (testIndex <= 0 || testIndex > static_cast<int>(m_funcs.size())) {
            _FVR_INFO("invalid test index!");
            return false;
        }

        m_funcs[testIndex]();

        return true;
    }

    void printTestCase(void)
    {
        std::cout << "===========================Robot Client Test "
                     "Case==============================="
                  << std::endl;
        for (auto test : m_testCase) {
            std::cout << "[" << test.first << "] " << test.second << "\n";
        }
    }

private:
    //! print vector content
    template <typename T>
    void printVectorInfo(const std::vector<T>& vec, const std::string& str,
        bool radToDeg = false)
    {
        _FVR_INFO("%s", str.c_str());

        for (auto& val : vec) {
            if (radToDeg == true) {
                _FVR_INFO("%f", val * 180.0 / M_PI);
            } else {
                _FVR_INFO("%f", val);
            }
        }

        _FVR_INFO("\n");
    }

    void registerTestCase(
        std::function<bool()> func, const std::string& testName)
    {
        m_funcs[m_funcs.size() + 1] = func;
        m_testCase[m_testCase.size() + 1] = testName;
    }

    //! map test index to test function
    std::map<int, std::function<bool()>> m_funcs;

    //! map test index to test name
    std::map<int, std::string> m_testCase;
};

#endif /* CLIENTTESTFUNCTION_HPP_ */
