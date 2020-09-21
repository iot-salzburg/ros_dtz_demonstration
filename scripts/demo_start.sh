
#!/bin/bash
# Felix Strohmeier / Armin Niedermueller - SALZBURG RESEARCH


echo "Starting DTZ - Robot / Storage / OPC Ua Script"

gnome-terminal -e "bash -c 'roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1'"
#sleep 5
gnome-terminal -e "roslaunch franka_gripper franka_gripper.launch robot_ip:=192.168.13.1"
sleep 3

rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"
gnome-terminal -e "bash -c 'roslaunch panda_moveit_config panda_moveit.launch'"
sleep 3
gnome-terminal -e "bash -c 'roslaunch panda_moveit_config moveit_rviz.launch'"
sleep 3
gnome-terminal -e "bash -c 'rosrun dtz_demonstration DTZ_LagerDemo'"
sleep 3
gnome-terminal -e "bash -c 'rosrun dtz_demonstration opc_ua_ros_server.py'"

sleep 3
echo "PRESS 'Continue'"

#Im Rviz Fenster -> obere Leiste "Panel" -> Add New Panel -> RvizVisualToolsgui (OK Drücken, nicht doppelklick)
#Dann unten auf "Continue"