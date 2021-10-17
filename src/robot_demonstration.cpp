//      _____         __        __                               ____                                        __
//     / ___/ ____ _ / /____   / /_   __  __ _____ ____ _       / __ \ ___   _____ ___   ____ _ _____ _____ / /_
//     \__ \ / __ `// //_  /  / __ \ / / / // ___// __ `/      / /_/ // _ \ / ___// _ \ / __ `// ___// ___// __ \
//    ___/ // /_/ // /  / /_ / /_/ // /_/ // /   / /_/ /      / _, _//  __/(__  )/  __// /_/ // /   / /__ / / / /
//   /____/ \__,_//_/  /___//_.___/ \__,_//_/    \__, /      /_/ |_| \___//____/ \___/ \__,_//_/    \___//_/ /_/
//                                              /____/
//   Salzburg Research ForschungsgesmbH
//
//   Armin Niedermueller

//  DTZ ROS Robot Demonstrator
//  The purpose of this program is to control the panda robot via ros
//  It gets its TODOs via a rostopic from the opcua panda server
//  It detects obstacles in its environment via 3D Cameras 

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_control/ErrorRecoveryActionGoal.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tuple>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdlib>

#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "PandaRobot.h"
#include "positions.h"


std::string global_moving{"false"};
std::string global_order_movement{"XX"};
std::string global_response{"none"};
std_msgs::String global_ros_response;
int global_order_pos{0};
int global_temperature;


struct MoveCommand {
    std::string position;
    std::string gripperBefore;
    std::string gripperAfter;
};

////////////////////////////////////////////////  ROBOT ROS METHODS  ///////////////////////////////////////////////////
void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    // create and initialize a stringstream object with our string
    std::stringstream ss{msg->data.c_str()};

    // split the string into to arguments
    ss >> global_order_movement;    // PO
    ss >> global_order_pos;         // 3

    ROS_INFO("Movement: %s", global_order_movement.c_str());
    ROS_INFO("Place: %d", global_order_pos);


} 

//////////////////////////////////////////////  M A I N  ///////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{


    ////////////////// INIT //////////////////
    // ROS INIT
    ros::init(argc, argv, "Robot_Demonstration");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("ros_opcua_order", 1000, chatterCallback);
    ros::Publisher pub = node_handle.advertise<std_msgs::String>("ros_opcua_response", 1000);
    ros::Publisher errorRecoverPub = node_handle.advertise<franka_control::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1000, true);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Get Own Robot Class
    PandaRobot *pandaRobot = PandaRobot::getInstance();

    pandaRobot->setSpeed(0.2);

    ///////////////// MAIN LOOP //////////////////
    try{

        while(ros::ok()) {

            ////////////////// RECEIVING ORDERS //////////////////
            //if (global_order_movement.compare("PS") == 0 || global_order_movement.compare("SO") == 0| global_order_movement.compare("PO") == 0) {

            if (global_order_movement.compare("PS") == 0 || global_order_movement.compare("SO") == 0 || global_order_movement.compare("SC") == 0 | global_order_movement.compare("PO") == 0) {

              //  ROS_INFO("Place: [%s]", global_order_pos);

                std::cout << "Place: " << global_order_pos << std::endl;

                // check if place is supported (must be between 1 and 9)
                if (global_order_pos < 1 || global_order_pos > 9) {
                    // std::cout << "Place not supported!" << std::endl;
                    global_order_movement = "XX";
                    global_order_pos = 0;
                    continue;
                }
            } else if (global_order_movement.compare("XX") == 0) {
                continue;
            }

            //std::cout << "Deliver order: " << global_order_movement << global_order_pos << std::endl;
            ROS_INFO("Deliver order: [%s %d]", global_order_movement.c_str(), global_order_pos);

            // publish error recovery (just in case)
            franka_control::ErrorRecoveryActionGoal empty{};
            errorRecoverPub.publish(empty);

            // publish state
            global_response = "Moving";
            // This is the Response from the robot if it is moving or not, necessary for the opcua master to wait until "stopped"
            // so that it could run the conveyor belt
            global_ros_response.data = global_response;
            pub.publish(global_ros_response);

            try{
                ////////////////// MOVEMENT OF ROBOT //////////////////


                pandaRobot->moveGripper("home");

                if (global_order_movement.compare("PO") == 0) {

                    // Move To Near Printer
                    pandaRobot->moveRobot(getPosition("near printer"));

                    // Move To Printer and grasp object
                    pandaRobot->moveRobot(getPosition("printer"), "open", "close");

                    // Move To Near Conveyor Belt
                    pandaRobot->moveRobot(getPosition("near conveyor belt"));

                    // Move To Conveyor Belt
                    pandaRobot->moveRobot(getPosition("conveyor belt"), "-", "open");

                    // Move To Initial Position
                    pandaRobot->moveRobot(getPosition("initial position"));

                } else if (global_order_movement.compare("PS") == 0) {

                    // Move To Near Printer
                    pandaRobot->moveRobot(getPosition("near printer"));

                    // Move To Printer and grasp object
                    pandaRobot->moveRobot(getPosition("printer"), "open", "close");

                    // Move To Near Storage
                    pandaRobot->moveRobot(getPosition("storage"));

                    // Move To Storage Place
                    pandaRobot->moveRobot(getPosition("storage place " + std::to_string(global_order_pos)), "-", "open");

                    // Move To Near Storage
                    pandaRobot->moveRobot(getPosition("storage"));

                    // Move To Initial Position
                    pandaRobot->moveRobot(getPosition("initial position"));

                } else if (global_order_movement.compare("SO") == 0) {
                    // Move To Near Storage
                    pandaRobot->moveRobot(getPosition("storage"));

                    // Move To Near Storage and Grasp Object before
                    pandaRobot->moveRobot(getPosition("near storage place " + std::to_string(global_order_pos)));

                    // Move To Storage Place
                    pandaRobot->moveRobot(getPosition("storage place " + std::to_string(global_order_pos)), "open", "close");

                    // Move To Near Storage and Grasp Object before
                    pandaRobot->moveRobot(getPosition("near storage place " + std::to_string(global_order_pos)));

                    // Move To Near Conveyor Belt
                    pandaRobot->moveRobot(getPosition("near conveyor belt"));

                    // Move To Conveyor Belt
                    pandaRobot->moveRobot(getPosition("conveyor belt"), "-", "open");

                    // Move To Initial Position
                    pandaRobot->moveRobot(getPosition("initial position"));

                } else if (global_order_movement.compare("SC") == 0) { // move from storage to cart

                    pandaRobot->moveRobot(getPosition("near storage place " + std::to_string(global_order_pos)));
/*                    std::vector<MoveCommand> moveCommands;
                    std::string pos = std::to_string(global_order_pos);
                    moveCommands.push_back({"cups init",      "-", "-"});
                    moveCommands.push_back({"cups storage",          "-", "-"});
                    moveCommands.push_back({"near cup " + pos,       "-", "-"});
                    moveCommands.push_back({"cup "      + pos,       "open", "close"});
                    moveCommands.push_back({"near cup " + pos,       "-", "-"});
                    //moveCommands.push_back({"cups storage",          "-", "-"});
                    moveCommands.push_back({"near cart position",    "-", "-"});
                    moveCommands.push_back({"cart position",         "-", "open"});
                    moveCommands.push_back({"near cart position",    "-", "-"});
                    moveCommands.push_back({"cups storage",          "close", "open"});                   

                    auto prevMove = moveCommands.begin();
                    for (auto moveCommand = moveCommands.begin(); moveCommand != moveCommands.end(); ++moveCommand) {
                        ROS_INFO("Move Command [%s],[%s],[%s]", 
                            moveCommand->position.c_str(),
                            moveCommand->gripperBefore.c_str(),
                            moveCommand->gripperAfter.c_str());
                        // in case we ran into an error before, recover it
                        errorRecoverPub.publish(empty);
                        try {
                            pandaRobot->moveRobot(getPosition(moveCommand->position), moveCommand->gripperBefore, moveCommand->gripperAfter);
                            prevMove = moveCommand;
                        } catch (const PandaRobot::MovementException& me) {
                            ROS_ERROR("Exception in move from '%s' to '%s'", prevMove->position.c_str(), moveCommand->position.c_str());
                            throw std::exception();
                        }
                    }
*/

                } else if (global_order_movement.compare("DD") == 0) {

                    // Move To Box Position on the left side of the desk
                    pandaRobot->moveRobot(getPosition("near desk left"));

                    // Move to box and grip it
                    //pandaRobot->moveRobot(getPosition("desk left"), "open", "close");

                    // Move To Box Position on the left side of the desk
                    //pandaRobot->moveRobot(getPosition("near desk left"));

                    // Move To Box Position on the right side of the desk
                    pandaRobot->moveRobot(getPosition("near desk right"));

                    // Move to box and grip it
                    //pandaRobot->moveRobot(getPosition("desk right"), "-", "open");

                    // Move To Box Position on the right side of the desk
                    //pandaRobot->moveRobot(getPosition("near desk right"), "close", "-");

                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception while executing order [%s %d]", global_order_movement.c_str(), global_order_pos);
                global_order_movement = "XX";
                global_order_pos = 0;

            //     era.sendGoal(goalError);
            }

            // publish state
            global_response = "Stopped";
            // This is the Response from the robot if it is moving or not, necessary for the opcua master to wait until "stopped"
            // so that it could run the conveyor belt
            global_ros_response.data = global_response;
            pub.publish(global_ros_response);

            // Reset Position, otherwise robot would move in an endless loop
            global_order_pos = 0;

            sleep(2);
        }

    } catch(const std::exception& e){
        std::cout << "Catched Exception: " << e.what() << " - stopping program" << std::endl;

        // Destroy Instance of own panda robot
        pandaRobot->destroyInstance();

        // Shut ROS Node down
        ros::shutdown();
        return 0;
    }
}

