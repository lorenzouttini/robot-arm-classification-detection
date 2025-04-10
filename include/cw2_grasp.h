#ifndef cw2_GRASP_H
#define cw2_GRASP_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "cw2_class.h"

namespace cw2_grasp
{

    // Open the gripper to prepare for grasp
    bool prepareGripperOpen(
        moveit::planning_interface::MoveGroupInterface &hand_group_,
        double gripper_open_);

    // Close the gripper to grasp the object
    bool closeGripperForGrasp(
        moveit::planning_interface::MoveGroupInterface &hand_group_,
        double gripper_closed_);

    // Release the object by opening the gripper
    bool releaseObject(
        moveit::planning_interface::MoveGroupInterface &hand_group_,
        double gripper_open_);

    // Compute the grasp pose
    geometry_msgs::PoseStamped computeGraspPose(
        const cw2::ObjectInfo &object,
    double lateral_offset = 0.05);

}

#endif // end of include guard for GRASP_H