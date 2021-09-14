#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka/exception.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <iostream>

#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_listener.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

//There functions call the action server provided by the gripper action. In this way, we can simplify the code inside the main function.
//In this demo, we only use grasp and move action. 
void grasp(){
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("franka_gripper/grasp", true);
    while(!grasp_client.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the franka_gripper/grasp server to come up");
    };

    franka_gripper::GraspGoal grasp_goal;
    grasp_goal.width = 0.04;
    grasp_goal.speed = 0.03; 
    grasp_goal.force = 10;
    grasp_goal.epsilon.inner = 0.005;
    grasp_goal.epsilon.outer = 0.005;

    ROS_INFO("Sending grasp goal to server");
    grasp_client.sendGoal(grasp_goal);

    franka_gripper::GraspResult grasp_result;
    if( grasp_client.waitForResult(ros::Duration(10.0))){
        ROS_INFO("The grasp action is finished");
        ROS_INFO("The grasp result is: %s", grasp_result.success?"success":"failed");
    }
    else{
        ROS_INFO("Failed to complete grasp action");
    }
} 

void move(){
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("franka_gripper/move", true);
    while(!move_client.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the franka_gripper/move server to come up");
    };

    franka_gripper::MoveGoal move_goal;
    move_goal.width = 0.08;
    move_goal.speed = 0.02;
    
    ROS_INFO("Sending move goal to server");
    move_client.sendGoal(move_goal);

    franka_gripper::MoveResult move_result;

    if(move_client.waitForResult(ros::Duration(10.0))){
        ROS_INFO("The move action is finished");
        ROS_INFO("The move result is: %s", move_result.success?"success":"failed");
    }
    else{
        ROS_INFO("Failed to complete move action");
    }
    
} 

void homing(){
    actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client("franka_gripper/homing", true);
    while(!homing_client.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the franka_gripper/homing server to come up");
    };

    franka_gripper::HomingGoal homing_goal;
    ROS_INFO_NAMED("test", "Sending homing goal to server");
    homing_client.sendGoal(homing_goal);

    franka_gripper::HomingResult homing_result;
    if(homing_client.waitForResult(ros::Duration(15.0))){
        ROS_INFO("The homing action is finished");
        ROS_INFO("The homing action is: %s", homing_result.success?"success":"failed");
    }
    else{
        ROS_INFO("Failed to complete homing action");
    }
   
}

void stop(){
    actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("franka_gripper/stop", true);
    while(!stop_client.waitForServer(ros::Duration(15.0))){
        ROS_INFO("Waiting for the franka_gripper/stop server to come up");
    };

    franka_gripper::StopGoal stop_goal;
    ROS_INFO("Sending stop goal to server");
    stop_client.sendGoal(stop_goal);

    franka_gripper::StopResult stop_result;
    if(stop_client.waitForResult(ros::Duration(10.0))){
        ROS_INFO("The stop action is finished");
        ROS_INFO("The stop action is: %s", stop_result.success?"success":"failed");
    }
    else{
        ROS_INFO("Failed to complete stop action");
        
    }
   

}

//This is the setted home configuration of robot arm 
std::vector<double> setHome(std::vector<double> joint_group_positions)
{
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -0.7;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = -2.0;
  joint_group_positions[4] = -0.0;
  joint_group_positions[5] = 1.5;
  joint_group_positions[6] = 0.7;
  
  return joint_group_positions;
}

//We create a function to get the arm move to the home configuration
void armHome(){
    std::vector<double> joint_group_positions;
    moveit::planning_interface::MoveGroupInterface::Plan  my_plan;
    moveit::planning_interface::MoveGroupInterface move_group_interface("panda_arm");
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools("panda_link0");
    const robot_state::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup("panda_arm");
    ros::Duration(0.5).sleep();

    move_group_interface.setJointValueTarget(setHome(joint_group_positions));
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
      moveit_visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      moveit_visual_tools.trigger();
      ros::Duration(1.5).sleep();
      moveit_visual_tools.deleteAllMarkers();
      move_group_interface.move();
    }
    else{
        ROS_INFO("Unable to find solution to move the arm to home configuration");
    }
}

//We create a function to get the arm move to the target_pose
void armMove(geometry_msgs::Pose target_pose){
    std::vector<double> joint_group_positions;
    moveit::planning_interface::MoveGroupInterface::Plan  my_plan;
    moveit::planning_interface::MoveGroupInterface move_group_interface("panda_arm");
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools("panda_link0");
    const robot_state::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup("panda_arm");
    ros::Duration(0.5).sleep();

    move_group_interface.setPoseTarget(target_pose);
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
      moveit_visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      moveit_visual_tools.trigger();
      ros::Duration(1.5).sleep();
      moveit_visual_tools.deleteAllMarkers();
      move_group_interface.move();
    }
    else{
        ROS_INFO("Failed to find solution to move the arm to the target pose");
    }
}

//Print out posestamped information
void printPoseStamped(geometry_msgs::PoseStamped poseStamped){
    ROS_INFO("The positionxyz is: [%f, %f, %f]",poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
    ROS_INFO("The quaternionxyzw is: [%f, %f, %f, %f]", poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z, poseStamped.pose.orientation.w);
}

//Get the geometry_msgs::PoseStamped of marker respect to world frame, w2m
geometry_msgs::PoseStamped pose_w2m(){
   
    tf::StampedTransform tf_transform_w2m;
    tf::TransformListener listener;
    ros::Duration(2.0).sleep();
    listener.waitForTransform("panda_link0", "aruco_marker_frame", ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("panda_link0", "aruco_marker_frame", ros::Time(0), tf_transform_w2m);

    geometry_msgs::TransformStamped msgs_transform_w2m;
    tf::transformStampedTFToMsg(tf_transform_w2m, msgs_transform_w2m);
    
    geometry_msgs::PoseStamped msgs_pose_w2m;
    msgs_pose_w2m.pose.orientation.w = msgs_transform_w2m.transform.rotation.w;
    msgs_pose_w2m.pose.orientation.x = msgs_transform_w2m.transform.rotation.x;
    msgs_pose_w2m.pose.orientation.y = msgs_transform_w2m.transform.rotation.y;
    msgs_pose_w2m.pose.orientation.z = msgs_transform_w2m.transform.rotation.z;
    msgs_pose_w2m.pose.position.x = msgs_transform_w2m.transform.translation.x;
    msgs_pose_w2m.pose.position.y = msgs_transform_w2m.transform.translation.y;
    msgs_pose_w2m.pose.position.z = msgs_transform_w2m.transform.translation.z;
    msgs_pose_w2m.header.frame_id = tf_transform_w2m.frame_id_;
    msgs_pose_w2m.header.stamp = tf_transform_w2m.stamp_;

    ROS_INFO("Print out the orginal marker pose respect to the panda_link0");
    printPoseStamped(msgs_pose_w2m);

    return msgs_pose_w2m;
}


//create a function to convert quaternion into rotation matirx
Eigen::Matrix3f quat2rot(geometry_msgs::Quaternion quat){
    Eigen::Matrix3f rotation_matrix;
    double w = quat.w;
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;

    rotation_matrix <<  1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y,
                        2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x,
                        2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y;
    return rotation_matrix;
};

//Because we want to grasp a known target(with marker on its surface) from a specified direction,
//we need to convert its orientation so that the gripper can grasp it from the direction where gripper's z-axis is perpendicular to the marker's xy-surface 
geometry_msgs::PoseStamped reorientate_pose_w2m(geometry_msgs::PoseStamped msgs_pose_w2m){
    geometry_msgs::Quaternion reorientate_quat;
    //In my case, I need to rotate -90deg around x-axis then rotate 90 around z-axis
    reorientate_quat.w =  0.7071068;
    reorientate_quat.x =  0.7071068;
    reorientate_quat.y = 0.0;
    reorientate_quat.z = 0.0;
    
    Eigen::Matrix3f reorientate_matrix = quat2rot(reorientate_quat);
    Eigen::Matrix3f pose_w2m_matrix = quat2rot(msgs_pose_w2m.pose.orientation);
    Eigen::Matrix3f after_reorientate_matrix = pose_w2m_matrix * reorientate_matrix;

    //Eigen: convert eigen matrix to quaternion
    Eigen::Quaternionf after_reorientate_quaternion(after_reorientate_matrix);

    geometry_msgs::PoseStamped target_pose;
    target_pose = msgs_pose_w2m;
    target_pose.pose.orientation.w =  after_reorientate_quaternion.w();
    target_pose.pose.orientation.x =  after_reorientate_quaternion.x();
    target_pose.pose.orientation.y =  after_reorientate_quaternion.y();
    target_pose.pose.orientation.z =  after_reorientate_quaternion.z();
    ROS_INFO("Print out the marker pose respect to the panda_link0 after reorientate");
    printPoseStamped(target_pose);

    return target_pose;
}

//Because the end-effector link of panda arm is panda_link8, when we send target_pose, we send target_pose to panda_link8
//However, we should make the gripper center(panda_K) to move to the target_pose instead of panda_link8
//So, we need to transform the target_pose so that panda_K can move to the real target_pose
//The new target_pose(transform from panda_link0 to new target_pose) for panda_link8 = panda_link0->panda_K * panda_K->panda_link8, where panda_link0->marker = panda_link0->panda_K
geometry_msgs::PoseStamped generate_target_pose(geometry_msgs::PoseStamped reorientate_pose_w2m){
    geometry_msgs::PoseStamped pose_02K = reorientate_pose_w2m;
    geometry_msgs::Quaternion quat_02K = pose_02K.pose.orientation;
    
    geometry_msgs::Quaternion quat_K28;
    quat_K28.x = 0.0;
    quat_K28.y = 0.0;
    quat_K28.z = 0.383;
    quat_K28.w = 0.924;

    Eigen::Matrix3f rotate_02K = quat2rot(quat_02K);
    Eigen::Matrix3f rotate_K28 = quat2rot(quat_K28);

    Eigen::Vector3f translation_02K(pose_02K.pose.position.x, pose_02K.pose.position.y, pose_02K.pose.position.z);
    Eigen::Vector3f translation_K28(0.0, 0.0, -0.10); 

    Eigen::Matrix3f rotate_028 = rotate_02K * rotate_K28;
    Eigen::Vector3f translation_028 = rotate_02K * translation_K28 + translation_02K;

    Eigen::Quaternionf quat_028(rotate_028);
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.orientation.w = quat_028.w();
    target_pose.pose.orientation.x = quat_028.x();
    target_pose.pose.orientation.y = quat_028.y();
    target_pose.pose.orientation.z = quat_028.z();
    target_pose.header = pose_02K.header;
    target_pose.pose.position.x = translation_028.x();
    target_pose.pose.position.y = translation_028.y();
    target_pose.pose.position.z = translation_028.z();

    return target_pose;

}


//After generate target_pose for panda_link8, we now generate pre_grasp_pose and after_grasp_pose
geometry_msgs::PoseStamped generate_pre_grasp_pose(geometry_msgs::PoseStamped target_pose){
    geometry_msgs::PoseStamped pre_grasp_pose;
 
    //we assume the pre_grasp_pose is the targer_pose translate 0.04 along its z-axis
    Eigen::Matrix3f rotate_02t = quat2rot(target_pose.pose.orientation); //rotation natrix of target_pose relative to panda_link0
    Eigen::Vector3f translation_02t(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    Eigen::Vector3f translation_t2p(0.0, 0.0, -0.04);
    Eigen::Vector3f translation_0tp = rotate_02t * translation_t2p + translation_02t;

    pre_grasp_pose = target_pose;
    pre_grasp_pose.pose.position.x = translation_0tp.x(); 
    pre_grasp_pose.pose.position.y = translation_0tp.y();
    pre_grasp_pose.pose.position.z = translation_0tp.z();

    return pre_grasp_pose;
}





int main(int argc, char** argv){
    ros::init(argc, argv, "franka_gripper_client");
    ros::start();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Debug: visualize the pose of aruco marker before/after reorientate
    geometry_msgs::PoseStamped marker_original_poseStamped = pose_w2m();
    geometry_msgs::Pose marker_original_pose = marker_original_poseStamped.pose;

    geometry_msgs::PoseStamped marker_after_poseStamped = reorientate_pose_w2m(marker_original_poseStamped);
    geometry_msgs::Pose marker_after_pose = marker_after_poseStamped.pose;

    geometry_msgs::PoseStamped target_poseStamped = generate_target_pose(marker_after_poseStamped);
    geometry_msgs::Pose target_pose = target_poseStamped.pose;

    geometry_msgs::PoseStamped pre_grasp_poseStamped = generate_pre_grasp_pose(target_poseStamped);
    geometry_msgs::Pose pre_grasp_pose = pre_grasp_poseStamped.pose;

    namespace rvt = rviz_visual_tools;
    rvt::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rvt::RvizVisualTools("panda_link0", "/rviz_visual_markers"));
    visual_tools_->deleteAllMarkers();

    ROS_INFO("Publish marker_reorientate_pose");
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishAxisLabeled(marker_after_pose, "marker_reorientate_pose",rvt::LARGE, rvt::RED);
    visual_tools_->trigger();
    ros::Duration(2.0).sleep();

    ROS_INFO("Publish target_pose");
    visual_tools_->publishAxisLabeled(target_pose,"target_pose", rvt::LARGE, rvt::GREEN);
    visual_tools_->trigger();
    ros::Duration(2.0).sleep();

    ROS_INFO("Publish pre_grasp_pose");
    visual_tools_->publishAxisLabeled(pre_grasp_pose,"pre_grasp_pose", rvt::LARGE, rvt::GREEN);
    visual_tools_->trigger();
    ros::Duration(2.0).sleep();

    //First we move the arm to the pre_grasp_pose
    armMove(pre_grasp_pose);
    //Move the gripper to the home position(fully open)
    move();
    
    //Next we move the arm to the target_pose(grasp_pose)
    armMove(target_pose);
    //Let the gripper grasps the obj with defined grasp parameters(max force, speed, desired width) 
    grasp();

    //Now the gripper should have grasped the obj, let's grasp the obj and move to the arm's home configuration
    armHome();
    
    //Last, we back to the pre_grasp_pose and release the obj
    armMove(pre_grasp_pose);
    move();
    
    //After releasing the obj, let the arm move back to the home configuration
    armHome();

    ros::shutdown();
    return 0;
}
