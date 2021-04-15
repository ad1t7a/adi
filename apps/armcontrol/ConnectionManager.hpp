// Copyright (C) Flexiv Ltd. 2020. All Rights Reserved.
//=============================================================================
/*!
 * \brief Robot client connection manager class
 *
 *  Created on: July 23, 2020
 *      Author: Yakun Ma
 */
//=============================================================================
#ifndef CONNECTIONMANAGER_HPP_
#define CONNECTIONMANAGER_HPP_

#define _FVR_INFO(...)                                                         \
    do {                                                                       \
        printf(__VA_ARGS__);                                                   \
        puts("");                                                              \
    } while (0);

#define _FVR_WARNING _FVR_INFO
#define _FVR_ERROR _FVR_INFO

#include "FvrRobotClient.hpp"
#include <memory>

using namespace fvr;

constexpr unsigned int k_robotDofs = 7;

//========================================================================
/*!
Clinet action by the connection manager

CLIENT_INIT_CONNECTION     :  initialize the communication with robot server
CLIENT_CHECK_CONNECTION    :  check if connection is lost
CLIENT_RECONNECT           :  reconnect with robot server
*/
//========================================================================
enum ClientAction
{
    CLIENT_INIT_CONNECTION,
    CLIENT_CHECK_CONNECTION,
    CLIENT_RECONNECT
};

class FvrRbotClient;

class ConnectionManager
{

public:
    ConnectionManager(std::shared_ptr<FvrRobotClient> robotClient,
        const std::string& serverAddr, const std::string& clientAddr)
    : m_robot(robotClient)
    , m_serverAddr(serverAddr)
    , m_clientAddr(clientAddr)
    , m_action(CLIENT_INIT_CONNECTION)
    {}

    virtual ~ConnectionManager() {}

    //! run the connection manager
    bool run()
    {
        // ------------------------initialize connection-----------------------
        if (m_action == CLIENT_INIT_CONNECTION) {

            if (m_robot != nullptr
                && m_robot->init(m_serverAddr, m_clientAddr, k_robotDofs)
                       == false) {
                _FVR_INFO("robot client failed to initialize");
                return false;
            }

            m_action = CLIENT_CHECK_CONNECTION;
        }
        // -----------------------monitor the connection status----------------
        else if (m_action == CLIENT_CHECK_CONNECTION) {

            if (m_robot->timeout()) {
                _FVR_INFO("robot client timeout, will close connection");
                m_robot->disconnect();
                m_action = CLIENT_RECONNECT;
            }

        }
        // ------------------------reconnect if connection lost-----------------
        else if (m_action == CLIENT_RECONNECT) {
            if (m_robot->connected() == false) {

                _FVR_INFO("connecting to robot server...");

                if (m_robot->connected() == false) {
                    m_robot->connect();
                } else {
                    m_action = CLIENT_CHECK_CONNECTION;
                }
            }
        }

        return true;
    }

    //! if robot is connected
    bool robotConnected() const
    {
        if (m_robot == nullptr) {
            return false;
        }

        return m_robot->connected();
    }

private:
    //! robot client pointer
    std::shared_ptr<FvrRobotClient> m_robot;

    //! robot server address
    std::string m_serverAddr;

    //! robot client address
    std::string m_clientAddr;

    //! connection action
    ClientAction m_action;
};

#endif /* CONNECTIONMANAGER_HPP_ */
