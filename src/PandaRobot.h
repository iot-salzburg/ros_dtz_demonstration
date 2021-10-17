
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/franka_gripper.h>

#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <string>

namespace rvt = rviz_visual_tools;

//--------------------------------------------------------------------------
// Singleton Class for Robot Control
//--------------------------------------------------------------------------
class PandaRobot{

private:    
    
    PandaRobot();  // Private Constructor
    ~PandaRobot(); // Private Destructor
    double speed;  // Robot Movement Speed
    static PandaRobot *instance; // Singleton Insance

    moveit::planning_interface::MoveGroupInterface::Plan my_plan; // Movement plan
    moveit::core::RobotStatePtr current_state; // Current Robot State
    

    // PLanning group
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group;
    const robot_state::JointModelGroup* joint_model_group;

    // Text pose for Rviz
    Eigen::Affine3d text_pose;
    moveit_visual_tools::MoveItVisualTools visual_tools;

    // Franka Gripper Goals
    franka_gripper::GraspGoal graspGoal;
    franka_gripper::StopGoal stopGoal;
    franka_gripper::MoveGoal moveGoal;
    franka_gripper::HomingGoal homingGoal; 

    // Action Clients
    actionlib::SimpleActionClient<franka_gripper::GraspAction> actionClientGrasp;
    actionlib::SimpleActionClient<franka_gripper::StopAction> actionClientStop;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> actionClientMove;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> actionClientHoming;
    // actionlib::SimpleActionClient<franka_control::ErrorRecoveryAction> era;


public:
    
    static PandaRobot* getInstance(); // Create a single instance if this class
    void setSpeed(double speed); // Getter & Setter
    void destroyInstance(); // destroy instance
    void moveGripper(std::string gripperCommand); // Gripper Commands

    // Move Robot
    bool moveRobot(std::vector<double> joints_goal, 
                   std::string firstGripperCommand = "-", 
                   std::string lastGripperCommand = "-");

    // Core Movement Function
    void moveGroupCoreFunction(std::vector<double> joints_goal,
                               const robot_state::JointModelGroup* joint_model_group,
                               moveit::planning_interface::MoveGroupInterface* move_group,
                               moveit_visual_tools::MoveItVisualTools visual_tools,
                               Eigen::Affine3d text_pose);
};




