#ifndef cw2_MOVEMENT_H
#define cw2_MOVEMENT_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>
#include <utility>
#include <cmath>

namespace cw2_movement
{
    // Move arm to the default ready position
    bool moveToReadyPosition(
        moveit::planning_interface::MoveGroupInterface &arm_group_,
        double planning_time = 25.0,
        double velocity_scaling = 0.5,
        double accel_scaling = 0.5,
        double eef_step = 0.01);

    // Plan and execute a Cartesian path to the specified waypoints
    bool planAndExecuteCartesianPath(
        moveit::planning_interface::MoveGroupInterface &arm_group_,
        const std::vector<geometry_msgs::Pose> &waypoints,
        const std::string &action_name,
        double eef_step = 0.01);

    // Fetch a set of observation poses for scanning the workspace
    std::vector<geometry_msgs::PoseStamped> getObservationPoses();

    // Move robot to an observation pose
    bool moveToObservationPose(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &pose,
        double eef_step = 0.01);

    // Move the arm to a specified target pose with a fallback to joint-space planning
    bool moveToTargetPose(
        moveit::planning_interface::MoveGroupInterface &arm_group_,
        const geometry_msgs::PoseStamped &target_pose,
        double eef_step = 0.01,
        double jump_threshold = 0.0);

    // Move to a pre-grasp pose above the target object
    bool moveToPreGraspPose(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &target_pose,
        double preferred_height_offset = 0.125,
        double eef_step = 0.01,
        int max_attempts = 5);

    // Lower the arm vertically to a target pose
    bool lowerTheArm(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &current_pose,
        const geometry_msgs::PoseStamped &target_pose,
        const std::string &action_name = "lowerTheArm",
        double eef_step = 0.005,
        double min_safe_height = 0.15);

    // Raise the arm vertically to a target pose
    bool raiseTheArm(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &current_pose,
        const geometry_msgs::PoseStamped &target_pose,
        const std::string &action_name = "raiseTheArm",
        double eef_step = 0.005,
        double max_safe_height = 0.8);

    // Move the arm to goal basket position with a hover height
    bool transportToBasket(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &basket_pose,
        double hover_height = 0.3, 
        double approach_distance = 0.05);
}

#endif // cw2_MOVEMENT_H
