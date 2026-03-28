// save as: src/fk_pose_finder.cpp
#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fk_pose_finder");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

    // ── Define joint angles to test (in radians) ──────────────────────────────
    // Adjust these angles to explore different poses
    vector<pair<string, vector<double>>> test_configs = {
        {"config_1", { 0.0,  0.0,   0.0,  0.0,  0.0}},
        {"config_2", { 0.0,  0.5,  -0.5,  0.5,  0.0}},
        {"config_3", { 0.5,  0.3,  -0.3,  0.3,  0.0}},
        {"config_4", {-0.5,  0.5,  -0.5,  0.5,  0.0}},
        {"config_5", { 0.0,  1.0,  -1.0,  1.0,  0.0}},
        {"config_6", { 0.0,  0.8,  -0.4,  0.3,  0.0}},
    };

    ROS_INFO_STREAM("============================================");
    ROS_INFO("Testing joint configurations → poses:");
    ROS_INFO_STREAM("============================================");

    for (auto& config : test_configs) {
        string name        = config.first;
        vector<double> joints = config.second;

        // Move to joint config
        dofbot.setJointValueTarget(joints);
        moveit::planning_interface::MoveItErrorCode code = dofbot.move();

        if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ros::Duration(0.5).sleep();

            // Get resulting pose
            geometry_msgs::PoseStamped pose = dofbot.getCurrentPose();

            // Convert to RPY
            tf::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            ROS_INFO_STREAM("--- " << name << " ---");
            ROS_INFO_STREAM("  Joints : ["
                << joints[0] << ", " << joints[1] << ", "
                << joints[2] << ", " << joints[3] << ", "
                << joints[4] << "]");
            ROS_INFO_STREAM("  Pos    : ["
                << pose.pose.position.x << ", "
                << pose.pose.position.y << ", "
                << pose.pose.position.z << "]");
            ROS_INFO_STREAM("  Quat   : ["
                << pose.pose.orientation.x << ", "
                << pose.pose.orientation.y << ", "
                << pose.pose.orientation.z << ", "
                << pose.pose.orientation.w << "]");
            ROS_INFO_STREAM("  RPY(deg): ["
                << roll  * (180.0/M_PI) << ", "
                << pitch * (180.0/M_PI) << ", "
                << yaw   * (180.0/M_PI) << "]");
        } else {
            ROS_WARN_STREAM("  ✘ " << name << " — joint move failed");
        }
    }

    ROS_INFO_STREAM("============================================");
    ROS_INFO("Done. Use the printed poses in your pick/place server.");

    ros::shutdown();
    return 0;
}