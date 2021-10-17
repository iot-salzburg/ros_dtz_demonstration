
#include "PandaRobot.h"

#include <ros/ros.h>
#include <exception>


//--------------------------------------------------------------------------
// local helpers
//--------------------------------------------------------------------------
struct MovementException : public std::exception {
    const char * what () const throw () {
        return "Movement cannot be executed";
    }
};

struct GoalToleranceExceededException : public std::exception {
    const char * what () const throw () {
        return "Goal tolerance exceeded";
    }
};

PandaRobot* PandaRobot::instance = nullptr;

//--------------------------------------------------------------------------
// Private Constructor (use getInstance() instead)
//--------------------------------------------------------------------------
PandaRobot::PandaRobot():
    
    // Action Clients
    actionClientGrasp("franka_gripper/grasp", true),
    actionClientStop("franka_gripper/stop", true),
    actionClientMove("franka_gripper/move", true),
    actionClientHoming("franka_gripper/homing", true),
    visual_tools("panda_link0"),
    speed(0.2),
    PLANNING_GROUP("panda_arm"),
    move_group(PLANNING_GROUP)   
{
    ROS_INFO("Action server started, sending goal.");

    // PLanning group 
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Add text inside Rviz
    text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Robot Demonstration", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Texts
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    //wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    actionClientGrasp.waitForServer(); //will wait for infinite time
    actionClientStop.waitForServer();
    actionClientMove.waitForServer();
    actionClientHoming.waitForServer();
    // era.waitForServer();};
    ROS_INFO("PandaRobot CTOR init ready, now waiting for orders.");
}        

//--------------------------------------------------------------------------
// Private Destructor (use destroyInstance() instead)
//--------------------------------------------------------------------------
PandaRobot::~PandaRobot(){};


//--------------------------------------------------------------------------
// Create a single instance if this class
//--------------------------------------------------------------------------
PandaRobot* PandaRobot::getInstance(){
    // Instanciate Instance or return the given instance
    if(instance == nullptr){
        instance = new PandaRobot;
    } 
    return instance;
}

//--------------------------------------------------------------------------
// destroy instance
//--------------------------------------------------------------------------
void PandaRobot::destroyInstance(){
    delete this;
    instance = nullptr;
}

//--------------------------------------------------------------------------
// Getter & Setter
//--------------------------------------------------------------------------
void PandaRobot::setSpeed(double speed){
        this->speed = speed;
}

//--------------------------------------------------------------------------
// Gripper Commands
//--------------------------------------------------------------------------
void PandaRobot::moveGripper(std::string gripperCommand){

    if (gripperCommand == "open"){
        actionClientStop.sendGoal(stopGoal);
        moveGoal.width = 0.08;
        moveGoal.speed = 0.1;
        actionClientMove.sendGoal(moveGoal);
    } else if (gripperCommand == "close"){
        graspGoal.width = 0.04;
        graspGoal.speed = 0.1;
        graspGoal.force = 60;
        graspGoal.epsilon.inner = 0.05;
        graspGoal.epsilon.outer = 0.05;
        actionClientGrasp.sendGoal(graspGoal);
    } else if (gripperCommand == "home"){
        actionClientHoming.sendGoal(homingGoal);
        sleep(3);
    }
}

//--------------------------------------------------------------------------
// Move Robot
//--------------------------------------------------------------------------
bool PandaRobot::moveRobot(std::vector<double> joints_goal, std::string firstGripperCommand, std::string lastGripperCommand){
            
    std::vector<double> joint_group_positions;

    // Move Gripper before Robot Movement
    ROS_DEBUG("moving gripper (%s)", firstGripperCommand);
    sleep(0.5);
    moveGripper(firstGripperCommand);
    sleep(0.5);
       
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions = joints_goal;

    moveGroupCoreFunction(joints_goal, joint_model_group, &move_group, visual_tools, text_pose);

    // Move Gripper before Robot Movement
    sleep(0.5);
    moveGripper(lastGripperCommand);
    sleep(0.5);
}

//--------------------------------------------------------------------------
// Core Movement Function
//--------------------------------------------------------------------------
void PandaRobot::moveGroupCoreFunction(std::vector<double> joints_goal,
                                       const robot_state::JointModelGroup* joint_model_group,
                                       moveit::planning_interface::MoveGroupInterface* move_group,
                                       moveit_visual_tools::MoveItVisualTools visual_tools,
                                       Eigen::Affine3d text_pose){
    move_group->setJointValueTarget(joints_goal);

/*  try {
            moveit::planning_interface::MoveItErrorCode planningResult = 2;
            float tolerance = 0.0001;
            while (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                move_group->setGoalTolerance(tolerance);
                planningResult = (move_group->plan(my_plan));
                ROS_INFO("Visualizing plan 1 (joint space goal). Planning %s", 
                (planningResult == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "successful" : "failed");

                if (planningResult == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED) {
                    ROS_WARN("Planning failed, increasing goal tolerance to %f", tolerance += 0.001);
                    if (tolerance > 0.01) {
                        throw GoalToleranceExceededException();
                    }
                }
            }
    } catch (const GoalToleranceExceededException& e) {
            ROS_ERROR("Planning failed with exception: %s", e.what());
            throw MovementException();
    }*/
        
    move_group->plan(my_plan);

    move_group->setMaxVelocityScalingFactor(speed);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group->move();
}




