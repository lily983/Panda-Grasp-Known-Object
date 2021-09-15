# Panda-Grasp-Known-Object
This repo uses the panda arm to grasp a known object(with Aruco marker). 

## Requirments
- Ubuntu 18.04
- franka_ros in catkin_ws (built from source)
- libfranka (built from source)
- aruco_ros in catkin_ws

## Installation
Download the "franka_gripper_client.cpp" in the folder catkin_ws/src/franka_ros/franka_gripper/src/
```
cd $path-to-catkin_ws/src/franka_ros/franka_gripper/src/
curl -o franka_gripper_client.cpp https://github.com/lily983/Panda-Grasp-Known-Object/blob/6fa6f59e592f49cb85c44b2366f78b60452e79bb/franka_gripper_client.cpp
```

Stay the directory in the terminal from the last step.

Delete the "CMakeLists.txt" and "package.xml" file in the folder catkin_ws/src/franka_ros/franka_gripper/

After that, install the two files from Github.
```
cd ..
rm CMakeLists.txt
rm package.xml
curl -o CMakeLists.txt 
