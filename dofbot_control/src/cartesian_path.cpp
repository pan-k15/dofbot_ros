#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>

using namespace std;

int main(int argc, char **argv)
{
    // ── ROS Init ──────────────────────────────────────────────────────────────
    ros::init(argc, argv, "cartesian_plan_cpp");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ── MoveGroup Setup ───────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");

    dofbot.allowReplanning(true);
    dofbot.setPlanningTime(50);
    dofbot.setNumPlanningAttempts(10);
    dofbot.setGoalJointTolerance(0.001);
    dofbot.setGoalPositionTolerance(0.001);
    dofbot.setGoalOrientationTolerance(0.001);
    dofbot.setGoalTolerance(0.001);
    dofbot.setMaxVelocityScalingFactor(1.0);
    dofbot.setMaxAccelerationScalingFactor(1.0);

    // ── Step 1: Move to named "down" pose ─────────────────────────────────────
    ROS_INFO("Moving to init pose: down");
    dofbot.setNamedTarget("down");
    dofbot.move();
    ros::Duration(0.5).sleep();

    // ── Step 2: Move to Cartesian start pose ──────────────────────────────────
    ROS_INFO("Moving to Cartesian start pose...");
    geometry_msgs::Pose pose;
    pose.position.x    =  0.0037618483876896;
    pose.position.y    =  0.1128923321179022;
    pose.position.z    =  0.3998656334826569;
    pose.orientation.x = -0.0042810851906468;
    pose.orientation.y = -0.0033330592972940;
    pose.orientation.z =  0.6827314913817025;
    pose.orientation.w =  0.7306492138509612;

    string link = dofbot.getEndEffectorLink();
    dofbot.setPoseTarget(pose, link);

    int index = 0;
    while (index <= 10) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const moveit::planning_interface::MoveItErrorCode &code = dofbot.plan(plan);
        if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("✔ Start pose plan success. Executing...");
            dofbot.execute(plan);
            break;
        } else {
            ROS_WARN_STREAM("✘ Start pose plan failed, attempt " << index + 1);
        }
        index++;
    }
    ros::Duration(5.0).sleep();

    // ── Step 3: Build Cartesian Waypoints ─────────────────────────────────────
    ROS_INFO("Building waypoints...");
    geometry_msgs::Pose start_pose =
        dofbot.getCurrentPose(dofbot.getEndEffectorLink()).pose;

    ROS_INFO_STREAM("Start pose: ["
        << start_pose.position.x << ", "
        << start_pose.position.y << ", "
        << start_pose.position.z << "]");

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);          // 1. start

    start_pose.position.z -= 0.01;
    waypoints.push_back(start_pose);          // 2. down 1cm

    start_pose.position.y -= 0.01;
    waypoints.push_back(start_pose);          // 3. back 1cm

    start_pose.position.z -= 0.01;
    waypoints.push_back(start_pose);          // 4. down 1cm

    start_pose.position.y += 0.01;
    waypoints.push_back(start_pose);          // 5. forward 1cm

    start_pose.position.z -= 0.01;
    waypoints.push_back(start_pose);          // 6. down 1cm

    start_pose.position.y -= 0.01;
    waypoints.push_back(start_pose);          // 7. back 1cm

    ROS_INFO_STREAM("Total waypoints: " << waypoints.size());

    // ── Step 4: Compute Cartesian Path ────────────────────────────────────────
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step       = 0.1;
    double fraction             = 0.0;
    int maxtries                = 1000;
    int attempts                = 0;

    ROS_INFO("Computing Cartesian path...");

    while (fraction < 1.0 && attempts < maxtries) {
        fraction = dofbot.computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if (attempts % 100 == 0) {
            ROS_INFO_STREAM("Still trying... attempt "
                << attempts << " fraction: " << fraction);
        }
    }

    // ── Step 5: Execute ───────────────────────────────────────────────────────
    if (fraction == 1.0) {
        ROS_INFO_STREAM("✔ Path computed successfully after "
            << attempts << " attempts. Executing...");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        moveit::planning_interface::MoveItErrorCode exec = dofbot.execute(plan);

        if (exec == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("✔ Cartesian path executed successfully!");
        } else {
            ROS_ERROR_STREAM("✘ Execution failed: " << exec);
        }
    } else {
        ROS_ERROR_STREAM("✘ Path planning failed. fraction: "
            << fraction << " after " << maxtries << " attempts.");
    }

    ros::shutdown();
    return 0;
}