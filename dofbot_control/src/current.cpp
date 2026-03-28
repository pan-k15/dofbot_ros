#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>   // ← add this
#include <tf/LinearMath/Matrix3x3.h>    // ← add this

using namespace std;

int main(int argc, char **argv)
{
    // ── ROS Init ──────────────────────────────────────────────────────────────
    ros::init(argc, argv, "dofbot_print_pose");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ── MoveGroup Setup ───────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

    // ── Print Frame Info ──────────────────────────────────────────────────────
    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("Planning Frame   : " << dofbot.getPlanningFrame());
    ROS_INFO_STREAM("End-Effector Link: " << dofbot.getEndEffectorLink());
    ROS_INFO_STREAM("============================================");

    // ── Get Current Pose ──────────────────────────────────────────────────────
    geometry_msgs::PoseStamped current_pose = dofbot.getCurrentPose();

    // ── Print Position ────────────────────────────────────────────────────────
    ROS_INFO_STREAM("--- Current Position ---");
    ROS_INFO_STREAM("  X: " << current_pose.pose.position.x);
    ROS_INFO_STREAM("  Y: " << current_pose.pose.position.y);
    ROS_INFO_STREAM("  Z: " << current_pose.pose.position.z);

    // ── Print Orientation (Quaternion) ────────────────────────────────────────
    ROS_INFO_STREAM("--- Current Orientation (Quaternion) ---");
    ROS_INFO_STREAM("  X: " << current_pose.pose.orientation.x);
    ROS_INFO_STREAM("  Y: " << current_pose.pose.orientation.y);
    ROS_INFO_STREAM("  Z: " << current_pose.pose.orientation.z);
    ROS_INFO_STREAM("  W: " << current_pose.pose.orientation.w);

    // ── Convert Quaternion → RPY ──────────────────────────────────────────────
    tf::Quaternion q(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // ── Print Orientation (RPY in Degrees) ───────────────────────────────────
    ROS_INFO_STREAM("--- Current Orientation (RPY in Degrees) ---");
    ROS_INFO_STREAM("  Roll : " << roll  * (180.0 / M_PI));
    ROS_INFO_STREAM("  Pitch: " << pitch * (180.0 / M_PI));
    ROS_INFO_STREAM("  Yaw  : " << yaw   * (180.0 / M_PI));

    // ── Print Joint Values ────────────────────────────────────────────────────
    ROS_INFO_STREAM("--- Current Joint Values ---");
    vector<double> joints = dofbot.getCurrentJointValues();
    for (size_t i = 0; i < joints.size(); i++) {
        ROS_INFO_STREAM("  Joint[" << i << "]: "
            << joints[i] << " rad  ("
            << joints[i] * (180.0 / M_PI) << " deg)");
    }

    ROS_INFO_STREAM("============================================");

    ros::shutdown();
    return 0;
}