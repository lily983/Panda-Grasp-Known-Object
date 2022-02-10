# Panda-Grasp-Known-Object
This repo uses the panda arm to grasp a known object (labeled with Aruco marker). 

## Hardware
- Panda arm from Franka Emika
- RGB camera (e.g. Realsense D435)

## Requirments
- Ubuntu 18.04
- franka_ros in catkin_ws (built from source)
- libfranka (built from source)
- panda_moveit_config 
- aruco_ros 
- rviz_visual_tools

## Installation
Download the "franka_gripper_client.cpp" in the folder catkin_ws/src/franka_ros/franka_gripper/src/
```
cd $path-to-catkin_ws/src/franka_ros/franka_gripper/src/
wget https://raw.githubusercontent.com/lily983/Panda-Grasp-Known-Object/main/franka_gripper_client.cpp
```

Stay the directory in the terminal from the last step.

Delete the "CMakeLists.txt" and "package.xml" file in the folder catkin_ws/src/franka_ros/franka_gripper/

After that, install the two files from Github.
```
cd ..
rm CMakeLists.txt
rm package.xml
wget https://raw.githubusercontent.com/lily983/Panda-Grasp-Known-Object/main/CMakeLists.txt
wget https://raw.githubusercontent.com/lily983/Panda-Grasp-Known-Object/main/package.xml
```
Now, we need to catkin_build the catkin_ws.
```
catkin_build
```

## Before Usage
Before using this repo, we need to do the following things:
- Install camera on a static place
- Handeye calibration (recommond repo: IFL_CAMP/easy_handeye url:https://github.com/IFL-CAMP/easy_handeye)
- Print out Aruco marker (https://chev.me/arucogen/) (chooes Dictionary: Original ArUco) (Remeber to check the printed marker size! Incorrect size will cause a huge error on calibration)
- Modify Aruco detection file in pkg aruco_ros

## Commands
- In the first terminal, launching the arm.launch file:
```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2
```
- In the second terminal, boardcasting the transformation between panda_link0 to camera:
```
rosrun tf static_transform_publisher x y z qx qy qz qw panda_link0 camera_color_frame
```
- In the third terminal, launching the camera:
```
roslaunch realsense2_camera rs_camera.launch
```
- In the fourth terminal, launching marker detect file:
```
roslaunch aurco_ros single.launch 
```

After all these setup, the robot arm is ready to grasp object.
- In the last terminal, running the file:
```
rosrun franka_gripper franka_gripper_client
```

