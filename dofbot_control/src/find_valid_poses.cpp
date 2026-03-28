// save as: src/find_valid_poses.cpp
#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_valid_poses");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ── Load robot model ──────────────────────────────────────────────────────
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(
        new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    const robot_state::JointModelGroup* joint_model_group =
        kinematic_model->getJointModelGroup("dofbot");

    ROS_INFO("Scanning workspace for valid IK solutions...");
    ROS_INFO_STREAM("============================================");

    int valid_count = 0;
    int total_count = 0;

    // ── Sweep XYZ workspace ───────────────────────────────────────────────────
    for (double x = -0.2; x <= 0.2; x += 0.05) {
        for (double y = -0.2; y <= 0.2; y += 0.05) {
            for (double z = 0.05; z <= 0.35; z += 0.05) {

                geometry_msgs::Pose test_pose;
                test_pose.position.x = x;
                test_pose.position.y = y;
                test_pose.position.z = z;

                // Test with neutral orientation
                test_pose.orientation.w = 1.0;
                test_pose.orientation.x = 0.0;
                test_pose.orientation.y = 0.0;
                test_pose.orientation.z = 0.0;

                // Try IK solution
                bool found_ik = kinematic_state->setFromIK(
                    joint_model_group,
                    test_pose,
                    0.1  // timeout
                );

                total_count++;

                if (found_ik) {
                    valid_count++;
                    ROS_INFO_STREAM("✔ VALID  x=" << x
                        << "  y=" << y
                        << "  z=" << z);
                }
            }
        }
    }

    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("Valid poses : " << valid_count << " / " << total_count);
    ROS_INFO_STREAM("============================================");

    ros::shutdown();
    return 0;
}