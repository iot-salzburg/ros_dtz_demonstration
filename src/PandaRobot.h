#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/franka_gripper.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <string>
#include <exception>

namespace rvt = rviz_visual_tools;

// Singleton Class for Robot Control
class PandaRobot{
public:
    struct MovementException : public std::exception {
        const char * what () const throw () {
            return "Movement cannot be executed";
        }
    };

private:    
   
    struct GoalToleranceExceededException : public std::exception {
        const char * what () const throw () {
            return "Goal tolerance exceeded";
        }
    };

    // Singleton Insance
    static PandaRobot *instance;

    // PLanning group
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group;
    const robot_state::JointModelGroup* joint_model_group;

    // Movement plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Robot Movement Speed
    double speed;

    // Current Robot State
    moveit::core::RobotStatePtr current_state;

    // Text pose for Rviz
    Eigen::Affine3d text_pose;
    moveit_visual_tools::MoveItVisualTools visual_tools;

    // Action Clients
    actionlib::SimpleActionClient<franka_gripper::GraspAction> actionClientGrasp;
    actionlib::SimpleActionClient<franka_gripper::StopAction> actionClientStop;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> actionClientMove;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> actionClientHoming;
    // actionlib::SimpleActionClient<franka_control::ErrorRecoveryAction> era;


    franka_gripper::GraspGoal graspGoal;
    franka_gripper::StopGoal stopGoal;
    franka_gripper::MoveGoal moveGoal;
    franka_gripper::HomingGoal homingGoal;  

    // Private Constructor
    PandaRobot():
    
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

    // Private Destructor
    ~PandaRobot(){};


public:
    // Create a single instance if this class
    static PandaRobot* getInstance(){
        // Instanciate Instance or return the given instance
        if(instance == nullptr){
            instance = new PandaRobot;
        } 

        return instance;
    }

    // destroy instance
    void destroyInstance(){
        delete this;
        instance = nullptr;
    }

    // Gripper Commands
    void moveGripper(std::string gripperCommand){
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


    // Move Robot
    bool moveRobot(std::vector<double> joints_goal, std::string firstGripperCommand = "-", std::string lastGripperCommand = "-"){
            
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

    // Core Movement Function
    void moveGroupCoreFunction(std::vector<double> joints_goal,
                               const robot_state::JointModelGroup* joint_model_group,
                               moveit::planning_interface::MoveGroupInterface* move_group,
                               moveit_visual_tools::MoveItVisualTools visual_tools,
                               Eigen::Affine3d text_pose){

        move_group->setJointValueTarget(joints_goal);

/*        try {
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

    // Getter & Setter
    void setSpeed(double speed){
        this->speed = speed;
    }

};

PandaRobot* PandaRobot::instance = nullptr;


