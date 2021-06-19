//=============================================================================
/*!
 * \brief This is a test program on RobotClient library
 *
 *  Created on: April 16, 2020
 *      Author: Yakun Ma
 */
//=============================================================================
#include "FvrRobotClient.hpp"
#include "ConnectionManager.hpp"
#include "ClientTestFunction.hpp"
#include <sched.h>
#include <sstream>

using namespace fvr;

int main(int argc, char* argv[])
{

    // set the thread prioirity
    struct sched_param param;
    param.sched_priority = 49;
    /*int rc = sched_setscheduler(getpid(), SCHED_FIFO, &param);
    if (rc) {
        std::cout << "failed to set schedule. Did you use sudo?\n";
        exit(1);
    }*/

    if (argc != 3) {
        std::cerr << "Usage: test_robotClient <host_ip> <local_ip>\n";
        return 1;
    }

    // create a robot client
    std::shared_ptr<FvrRobotClient> robot = std::make_shared<FvrRobotClient>();

    // create connection manager
    ConnectionManager robotConnection(robot, argv[1], argv[2]);

    // create thread for running connection
    std::thread connection([&]() {
        while (true) {

            if (robotConnection.run() != true) {
                return;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    ClientTestFunction clientTester(robot);
    clientTester.printTestCase();

    std::string cmdStr;
    unsigned int cmdNum;

    // get test index
    while (std::cin >> cmdStr) {
        // receive \n from keyboard
        getchar();

        std::stringstream ss(cmdStr.c_str());
        ss >> cmdNum;

        if (robotConnection.robotConnected() == false) {
            _FVR_INFO("did not connect to robot server!");
            continue;
        }

        // run corresponding example
        if (clientTester(cmdNum) != true) {
            _FVR_INFO("Failed to run test case");
            break;
        }

        // print menu for next test case
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        clientTester.printTestCase();
    }

    return 0;
}
