#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "dofbot_control/MovePose.h"   // ← your package name / service

using namespace std;

// ── Global MoveGroup pointer (initialized once, reused across calls) ──────────
moveit::planning_interface::MoveGroupInterface* dofbot_group = nullptr;

// ── Service Callback ──────────────────────────────────────────────────────────
bool movePoseCallback(dofbot_control::MovePose::Request  &req,
                      dofbot_control::MovePose::Response &res)
{
    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("Service called!");
    ROS_INFO_STREAM("Target Position   : ["
        << req.x << ", " << req.y << ", " << req.z << "]");
    ROS_INFO_STREAM("Target Quaternion : ["
        << req.qx << ", " << req.qy << ", "
        << req.qz << ", " << req.qw << "]");
    ROS_INFO_STREAM("============================================");

    // ── Build target pose ─────────────────────────────────────────────────────
    geometry_msgs::Pose target_pose;
    target_pose.position.x    = req.x;
    target_pose.position.y    = req.y;
    target_pose.position.z    = req.z;
    target_pose.orientation.x = req.qx;
    target_pose.orientation.y = req.qy;
    target_pose.orientation.z = req.qz;
    target_pose.orientation.w = req.qw;

    // ── Set target ────────────────────────────────────────────────────────────
    dofbot_group->setPoseTarget(target_pose, dofbot_group->getEndEffectorLink());

    // ── Plan & Execute ────────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = false;

    for (int i = 0; i < 10; i++) {
        ROS_INFO_STREAM("Planning attempt " << (i + 1) << " / 10");

        moveit::planning_interface::MoveItErrorCode code = dofbot_group->plan(plan);

        if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("✔ Plan found! Executing...");
            plan_success = true;

            moveit::planning_interface::MoveItErrorCode exec = dofbot_group->execute(plan);

            if (exec == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_INFO("✔ Execution successful!");
                res.success = true;
                res.message = "Motion executed successfully.";
            } else {
                ROS_ERROR_STREAM("✘ Execution failed, code: " << exec);
                res.success = false;
                res.message = "Plan succeeded but execution failed.";
            }
            break;
        } else {
            ROS_WARN_STREAM("✘ Plan attempt " << (i + 1) << " failed.");
        }
    }

    if (!plan_success) {
        ROS_ERROR("✘ All planning attempts failed!");
        res.success = false;
        res.message = "All planning attempts failed. Check pose validity.";
    }

    // ── Print current pose after move ─────────────────────────────────────────
    geometry_msgs::PoseStamped cur = dofbot_group->getCurrentPose();
    tf::Quaternion q(
        cur.pose.orientation.x,
        cur.pose.orientation.y,
        cur.pose.orientation.z,
        cur.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM("--- Actual pose after move ---");
    ROS_INFO_STREAM("  X: " << cur.pose.position.x
                  << "  Y: " << cur.pose.position.y
                  << "  Z: " << cur.pose.position.z);
    ROS_INFO_STREAM("  Roll : " << roll  * (180.0 / M_PI)
                  << "  Pitch: " << pitch * (180.0 / M_PI)
                  << "  Yaw  : " << yaw   * (180.0 / M_PI));
    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("Ready for next call.");

    return true;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_pose_server");
    ros::NodeHandle n;

    // !! Use 2 threads: 1 for MoveIt, 1 for service calls
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // ── Init MoveGroup once ───────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    dofbot.allowReplanning(true);
    dofbot.setPlanningTime(10.0);
    dofbot.setNumPlanningAttempts(20);
    dofbot.setGoalPositionTolerance(0.05);
    dofbot.setGoalOrientationTolerance(0.05);
    dofbot.setMaxVelocityScalingFactor(0.5);
    dofbot.setMaxAccelerationScalingFactor(0.5);

    // ── Assign to global pointer ──────────────────────────────────────────────
    dofbot_group = &dofbot;

    ROS_INFO_STREAM("Planning Frame   : " << dofbot.getPlanningFrame());
    ROS_INFO_STREAM("End-Effector Link: " << dofbot.getEndEffectorLink());

    // ── Advertise service ─────────────────────────────────────────────────────
    ros::ServiceServer service = n.advertiseService("move_pose", movePoseCallback);
    ROS_INFO("✔ move_pose service is ready and waiting for calls...");

    ros::waitForShutdown();
    return 0;
}