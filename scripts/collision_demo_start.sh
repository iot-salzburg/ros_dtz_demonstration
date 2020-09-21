# Armin Niedermüller
# Salzburg Research
# armin-niedermueller.net

# SCRIPT TO START A DEMO USECASE FOR COLLISION-AWARE ROBOT PATH-PLANNING

echo "Armin Niedermüller"
echo "Salzburg Research"
echo "armin-niedermueller.net"
echo ""

# Start the 3D camera ROS nodes
echo "Starting Intel D435 3D Camera Nodes"
gnome-terminal -e "roslaunch registration_3d start_3d_cams.launch"
sleep 1

# Two Options to align the images of the 3D Cameras - Use ICP Alignment if camera angles are changed

        # Align their images with the ICP algorithm
        #echo "Starting ICP Alignment"
        #gnome-terminal -e "bash -c 'rosrun registration_3d preprocess_align_publish allsteps=true algorithm=nlicp /cam_1/depth/color/points /cam_2/depth/color/points'"

        # Aligh their images manually
        gnome-terminal -e "rosrun tf static_transform_publisher 0.02 1.49 1.3 0.002 -0.599 0.800 0.066 cam_2_depth_optical_frame cam_1_depth_optical_frame 2"
        sleep 1

# Align both images to the robot model
echo "Aligning 3D Cameras to Desk"
gnome-terminal -e "rosrun tf static_transform_publisher 0.60 1.1 1.5 0 0.1 2.44 world cam_2_depth_optical_frame 3" 
gnome-terminal -e "rosrun tf static_transform_publisher 0 0 0 0 0 0 world panda_link0 4" 

# Filter the point clouds
echo "Preprocess Point Clouds"
gnome-terminal -e "bash -c 'rosrun collision_avoidance pointcloud_preprocessing downsampling=true outlier=true passthrough=true'"
sleep 1

# Start the robot control node
gnome-terminal -e "bash -c 'roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1'"
sleep 1

# Reset communication errors of the robot
rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"
gnome-terminal -e "bash -c 'roslaunch panda_moveit_config panda_moveit.launch'"
sleep 1

# Start MoveIt Rviz
echo "Start MoveIt"
gnome-terminal -e "bash -c 'roslaunch collision_avoidance collision_avoidance.launch'"
sleep 1

# Start the OCP-UA server for robot control via OPC-UA
gnome-terminal -e "bash -c 'rosrun dtz_demonstration opc_ua_ros_server.py'"

# Run the usecase application
gnome-terminal -e "bash -c 'rosrun dtz_demonstration robot_demonstration'"

# End
echo "PRESS 'Continue'"





