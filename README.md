# ros_dtz_demonstration
[Youtube Video](https://raw.githubusercontent.com/nerovalerius/collision_avoidance/master/images/full_desk.jpg)](https://www.youtube.com/watch?v=LQPS--bnvQY)

This package realizes a software prototype to let a franka panda cobot (collaborative robot) recognize its surroundings with two 3D cameras and avoid obstacles.
The cobot is controlled via ROS and Ubuntu.The cobot's surroundings are sensed with two Intel D435 3D-Cameras, which are mounted above a human-robot-collaborative (HRC) workspace. Their point cloud streams are first semi-automatically aligned with the iterative closest point algorithm (ICP), such that the final 3D point cloud stream of the HRC workspace shows the whole workspace almost without any masked areas. Afterwards, the point clouds are converted into Octomaps and visualized inside Rviz together with the 3D model of the cobot.

## Prerequisites
- [Ubuntu Xenial](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic](http://wiki.ros.org/kinetic)

## Installation
- Install Ubuntu and ROS.
- Create a [catkin](http://wiki.ros.org/catkin) workspace (see [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).
- Clone this repository into the workspace's `src` directory.
- Follow the instructions in [here](https://github.com/nerovalerius/collision_avoidance) 


## Usage
Start the Demonstration Scenario with Collision Avoidance:
```sh
rosrun dtz_demonstration collision_demo_start.sh
```
OR Start the Demonstration Scenarion without Collision Avoidance:
```sh
rosrun dtz_demonstration demo_start.sh
```

The robot is then either controllable via ros topics OR via OPC-UA:
- Via ROS Topics:
    ```sh
    rostopic pub /ros_opcua_order std_msgs/String "DD"
    ```
    Possible Commands are:
    - "DD" for the Collision Avoidance Scenario
    - "SO X" to let the robot take something out of the storage, X stands for a Number between 0-9
    - "PO" to let the robot take something from the 3D Printer

- Via OPC-UA:
    -Connect to the OPC-UA Server and use the same Strings ("DD", "SO X", "PO") inside the MoveRobotRos Meethod








