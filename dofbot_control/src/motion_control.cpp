#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;

int main(int argc, char **argv)
{
    // ── ROS Init ──────────────────────────────────────────────────────────────
    ros::init(argc, argv, "dofbot_move_to_pose");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ── MoveGroup Setup ───────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

    dofbot.allowReplanning(true);
    dofbot.setPlanningTime(10.0);
    dofbot.setNumPlanningAttempts(20);
    dofbot.setGoalPositionTolerance(0.05);
    dofbot.setGoalOrientationTolerance(0.05);
    dofbot.setMaxVelocityScalingFactor(0.5);
    dofbot.setMaxAccelerationScalingFactor(0.5);

    ROS_INFO_STREAM("Planning Frame   : " << dofbot.getPlanningFrame());
    ROS_INFO_STREAM("End-Effector Link: " << dofbot.getEndEffectorLink());

    // ── Define Target Pose (from your current pose log) ───────────────────────
    geometry_msgs::Pose target_pose;

    // Position
    target_pose.position.x = -0.0670295;
    target_pose.position.y = -0.00339677;
    target_pose.position.z =  0.303328;

    // Orientation (Quaternion — copied directly from log)
    target_pose.orientation.x = -0.230018;
    target_pose.orientation.y = -0.38372;
    target_pose.orientation.z =  0.500876;
    target_pose.orientation.w =  0.740928;

    // ── Print Target ──────────────────────────────────────────────────────────
    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("--- Target Position ---");
    ROS_INFO_STREAM("  X: " << target_pose.position.x);
    ROS_INFO_STREAM("  Y: " << target_pose.position.y);
    ROS_INFO_STREAM("  Z: " << target_pose.position.z);
    ROS_INFO_STREAM("--- Target Orientation (Quaternion) ---");
    ROS_INFO_STREAM("  X: " << target_pose.orientation.x);
    ROS_INFO_STREAM("  Y: " << target_pose.orientation.y);
    ROS_INFO_STREAM("  Z: " << target_pose.orientation.z);
    ROS_INFO_STREAM("  W: " << target_pose.orientation.w);
    ROS_INFO_STREAM("============================================");

    // ── Set Target & Plan ─────────────────────────────────────────────────────
    dofbot.setPoseTarget(target_pose, dofbot.getEndEffectorLink());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    for (int i = 0; i < 10; i++) {
        ROS_INFO_STREAM("Planning attempt " << (i + 1) << " / 10");

        moveit::planning_interface::MoveItErrorCode code = dofbot.plan(plan);

        if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("✔ Plan found! Executing...");
            success = true;

            moveit::planning_interface::MoveItErrorCode exec = dofbot.execute(plan);
            if (exec == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_INFO("✔ Execution successful!");
            } else {
                ROS_ERROR_STREAM("✘ Execution failed with code: " << exec);
            }
            break;
        } else {
            ROS_WARN_STREAM("✘ Plan failed (attempt " << (i + 1) << ")");
        }
    }

    // ── Fallback: print current pose if failed ────────────────────────────────
    if (!success) {
        ROS_ERROR("✘ All planning attempts failed!");
        ROS_WARN("Printing current pose for diagnosis...");

        geometry_msgs::PoseStamped cur = dofbot.getCurrentPose();
        ROS_INFO_STREAM("Current X: " << cur.pose.position.x);
        ROS_INFO_STREAM("Current Y: " << cur.pose.position.y);
        ROS_INFO_STREAM("Current Z: " << cur.pose.position.z);

        tf::Quaternion q(
            cur.pose.orientation.x,
            cur.pose.orientation.y,
            cur.pose.orientation.z,
            cur.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("Current Roll : " << roll  * (180.0 / M_PI));
        ROS_INFO_STREAM("Current Pitch: " << pitch * (180.0 / M_PI));
        ROS_INFO_STREAM("Current Yaw  : " << yaw   * (180.0 / M_PI));
    }

    ros::shutdown();
    return 0;
}