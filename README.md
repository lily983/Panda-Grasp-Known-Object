# Panda-Grasp-Known-Object
This repo uses the panda arm to grasp a known object(with Aruco marker). 

## Hardware
- Panda arm from Franka Emika
- RGB camera (e.g. Realsense D435)

## Requirments
- Ubuntu 18.04
- franka_ros in catkin_ws (built from source)
- libfranka (built from source)
- aruco_ros in catkin_ws

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
- Print out Aruco marker (https://chev.me/arucogen/)
- Modify Aruco detection file in pkg aruco_ros

Example of modified Aruco detection file for camera (Realsense D435) and marker (ID:55, size:0.07, Dictionary: Original ArUco) has been uploaded.

## Commands

