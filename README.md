# ros_dtz_demonstration

Tested with libfranka 0.5.0 and franka_ros 0.7.0

## print_franka_joints
simply prints the 7 joint values of the robot's current position on standard out

## config
franka::RealtimeConfig::kIgnore must be set when instantiating franka::Robot class.
Otherwise the robot just runs with an FULL_PREEMPT_RT Kernel. Means: No Nvidia Drivers.
This hast to be edited inside franka_ros/franka_control/franka_control_node.cpp. 




