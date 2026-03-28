#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "dofbot_control/Place.h"

using namespace std;

// ── Global MoveGroup pointers ─────────────────────────────────────────────────
moveit::planning_interface::MoveGroupInterface* arm_group     = nullptr;
moveit::planning_interface::MoveGroupInterface* gripper_group = nullptr;

// ── Helper: Move Arm to Pose ──────────────────────────────────────────────────
bool moveArmToPose(geometry_msgs::Pose target_pose, const string& step_name)
{
    ROS_INFO_STREAM("--- [" << step_name << "] Moving arm ---");
    ROS_INFO_STREAM("  Target: ["
        << target_pose.position.x << ", "
        << target_pose.position.y << ", "
        << target_pose.position.z << "]");

    arm_group->setPoseTarget(target_pose, arm_group->getEndEffectorLink());

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    for (int i = 0; i < 10; i++) {
        ROS_INFO_STREAM("  Planning attempt " << (i + 1) << " / 10");

        moveit::planning_interface::MoveItErrorCode code = arm_group->plan(plan);

        if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("  ✔ Plan found! Executing [" << step_name << "]...");

            moveit::planning_interface::MoveItErrorCode exec = arm_group->execute(plan);

            if (exec == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_INFO_STREAM("  ✔ [" << step_name << "] done.");
                return true;
            } else {
                ROS_ERROR_STREAM("  ✘ Execution failed: " << exec);
                return false;
            }
        } else {
            ROS_WARN_STREAM("  ✘ Plan attempt " << (i + 1) << " failed.");
        }
    }

    ROS_ERROR_STREAM("  ✘ All planning attempts failed for [" << step_name << "]");
    return false;
}

// ── Helper: Control Gripper ───────────────────────────────────────────────────
bool controlGripper(const string& state)
{
    ROS_INFO_STREAM("--- Gripper: " << state << " ---");

    gripper_group->setNamedTarget(state);

    moveit::planning_interface::MoveItErrorCode code = gripper_group->move();

    if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM("  ✔ Gripper " << state << " successful.");
        return true;
    } else {
        ROS_ERROR_STREAM("  ✘ Gripper " << state << " failed: " << code);
        return false;
    }
}

// ── Helper: Move to Home ──────────────────────────────────────────────────────
bool moveToHome()
{
    ROS_INFO("--- Moving to Home ---");

    arm_group->setNamedTarget("home");

    moveit::planning_interface::MoveItErrorCode code = arm_group->move();

    if (code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("  ✔ Reached home position.");
        return true;
    } else {
        ROS_ERROR_STREAM("  ✘ Failed to reach home: " << code);
        return false;
    }
}

// ── Service Callback ──────────────────────────────────────────────────────────
bool placeCallback(dofbot_control::Place::Request  &req,
                   dofbot_control::Place::Response &res)
{
    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("Place service called!");
    ROS_INFO_STREAM("Place Position   : ["
        << req.x << ", " << req.y << ", " << req.z << "]");
    ROS_INFO_STREAM("Place Quaternion : ["
        << req.qx << ", " << req.qy << ", "
        << req.qz << ", " << req.qw << "]");
    ROS_INFO_STREAM("============================================");

    // ── Step 1: Move to pre-place (above place position) ─────────────────────
    ROS_INFO("=== STEP 1: Move to Pre-Place Pose ===");
    geometry_msgs::Pose pre_place_pose;
    pre_place_pose.position.x    = req.x;
    pre_place_pose.position.y    = req.y;
    pre_place_pose.position.z    = req.z + 0.05;  // 5cm above place position
    pre_place_pose.orientation.x = req.qx;
    pre_place_pose.orientation.y = req.qy;
    pre_place_pose.orientation.z = req.qz;
    pre_place_pose.orientation.w = req.qw;

    if (!moveArmToPose(pre_place_pose, "Pre-Place")) {
        res.success = false;
        res.message = "STEP 1 FAILED: Could not reach pre-place pose.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 2: Move down to exact place position ─────────────────────────────
    ROS_INFO("=== STEP 2: Move to Place Pose ===");
    geometry_msgs::Pose place_pose;
    place_pose.position.x    = req.x;
    place_pose.position.y    = req.y;
    place_pose.position.z    = req.z;  // exact place position
    place_pose.orientation.x = req.qx;
    place_pose.orientation.y = req.qy;
    place_pose.orientation.z = req.qz;
    place_pose.orientation.w = req.qw;

    if (!moveArmToPose(place_pose, "Place")) {
        res.success = false;
        res.message = "STEP 2 FAILED: Could not reach place pose.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 3: Open gripper (release object) ────────────────────────────────
    ROS_INFO("=== STEP 3: Open Gripper (Release Object) ===");
    if (!controlGripper("open")) {
        res.success = false;
        res.message = "STEP 3 FAILED: Could not open gripper.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 4: Retreat (move back up) ───────────────────────────────────────
    ROS_INFO("=== STEP 4: Retreat ===");
    geometry_msgs::Pose retreat_pose = place_pose;
    retreat_pose.position.z += 0.08;  // retreat 8cm up

    if (!moveArmToPose(retreat_pose, "Retreat")) {
        res.success = false;
        res.message = "STEP 4 FAILED: Could not retreat.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 5: Back to Home ──────────────────────────────────────────────────
    ROS_INFO("=== STEP 5: Back to Home ===");
    if (!moveToHome()) {
        res.success = false;
        res.message = "STEP 5 FAILED: Could not return to home.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }

    // ── Done ──────────────────────────────────────────────────────────────────
    res.success = true;
    res.message = "Place successful! Object released and arm returned home.";

    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("✔ " << res.message);
    ROS_INFO_STREAM("Ready for next call.");
    ROS_INFO_STREAM("============================================");

    return true;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    ros::init(argc, argv, "place_server");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // ── Init arm move group ───────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm("dofbot");
    arm.allowReplanning(true);
    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(20);
    arm.setGoalPositionTolerance(0.05);
    arm.setGoalOrientationTolerance(0.05);
    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);
    arm_group = &arm;

    // ── Init gripper move group ───────────────────────────────────────────────
    // ⚠️ Replace "gripper" with your actual SRDF gripper group name
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);
    gripper_group = &gripper;

    ROS_INFO_STREAM("Arm Planning Frame   : " << arm.getPlanningFrame());
    ROS_INFO_STREAM("Arm End-Effector Link: " << arm.getEndEffectorLink());

    // ── Advertise service ─────────────────────────────────────────────────────
    ros::ServiceServer service = n.advertiseService("place", placeCallback);

    ROS_INFO_STREAM("============================================");
    ROS_INFO("✔ place_server is ready and waiting for calls...");
    ROS_INFO_STREAM("============================================");

    ros::waitForShutdown();
    return 0;
}