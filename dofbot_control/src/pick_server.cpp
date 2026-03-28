#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "dofbot_control/Pick.h"

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

    // Use named target: "open" or "close"
    // These named states must be defined in your SRDF
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

// ── Service Callback ──────────────────────────────────────────────────────────
bool pickCallback(dofbot_control::Pick::Request  &req,
                  dofbot_control::Pick::Response &res)
{
    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("Pick service called!");
    ROS_INFO_STREAM("Object Position   : ["
        << req.x << ", " << req.y << ", " << req.z << "]");
    ROS_INFO_STREAM("Object Quaternion : ["
        << req.qx << ", " << req.qy << ", "
        << req.qz << ", " << req.qw << "]");
    ROS_INFO_STREAM("============================================");

    // ── Step 1: Open gripper ──────────────────────────────────────────────────
    ROS_INFO("=== STEP 1: Open Gripper ===");
    if (!controlGripper("open")) {
        res.success = false;
        res.message = "STEP 1 FAILED: Could not open gripper.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep(); // short settle time

    // ── Step 2: Move arm to pre-grasp (slightly above object) ─────────────────
    ROS_INFO("=== STEP 2: Move to Pre-Grasp Pose ===");
    geometry_msgs::Pose pre_grasp_pose;
    pre_grasp_pose.position.x    = req.x;
    pre_grasp_pose.position.y    = req.y;
    pre_grasp_pose.position.z    = req.z + 0.05; // 5cm above object
    pre_grasp_pose.orientation.x = req.qx;
    pre_grasp_pose.orientation.y = req.qy;
    pre_grasp_pose.orientation.z = req.qz;
    pre_grasp_pose.orientation.w = req.qw;

    if (!moveArmToPose(pre_grasp_pose, "Pre-Grasp")) {
        res.success = false;
        res.message = "STEP 2 FAILED: Could not reach pre-grasp pose.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 3: Move arm down to grasp pose ───────────────────────────────────
    ROS_INFO("=== STEP 3: Move to Grasp Pose ===");
    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x    = req.x;
    grasp_pose.position.y    = req.y;
    grasp_pose.position.z    = req.z;  // exact object position
    grasp_pose.orientation.x = req.qx;
    grasp_pose.orientation.y = req.qy;
    grasp_pose.orientation.z = req.qz;
    grasp_pose.orientation.w = req.qw;

    if (!moveArmToPose(grasp_pose, "Grasp")) {
        res.success = false;
        res.message = "STEP 3 FAILED: Could not reach grasp pose.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 4: Close gripper (grab object) ───────────────────────────────────
    ROS_INFO("=== STEP 4: Close Gripper (Grab Object) ===");
    if (!controlGripper("close")) {
        res.success = false;
        res.message = "STEP 4 FAILED: Could not close gripper.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }
    ros::Duration(0.5).sleep();

    // ── Step 5: Lift object up ────────────────────────────────────────────────
    ROS_INFO("=== STEP 5: Lift Object ===");
    geometry_msgs::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.05; // lift 5cm

    if (!moveArmToPose(lift_pose, "Lift")) {
        res.success = false;
        res.message = "STEP 5 FAILED: Could not lift object.";
        ROS_ERROR_STREAM(res.message);
        return true;
    }

    // ── Done ──────────────────────────────────────────────────────────────────
    res.success = true;
    res.message = "Pick successful! Object grabbed and lifted.";

    ROS_INFO_STREAM("============================================");
    ROS_INFO_STREAM("✔ " << res.message);
    ROS_INFO_STREAM("Ready for next call.");
    ROS_INFO_STREAM("============================================");

    return true;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_server");
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
    // ⚠️ Replace "gripper" with your actual gripper group name in SRDF
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);
    gripper_group = &gripper;

    ROS_INFO_STREAM("Arm Planning Frame   : " << arm.getPlanningFrame());
    ROS_INFO_STREAM("Arm End-Effector Link: " << arm.getEndEffectorLink());

    // ── Advertise service ─────────────────────────────────────────────────────
    ros::ServiceServer service = n.advertiseService("pick", pickCallback);

    ROS_INFO_STREAM("============================================");
    ROS_INFO("✔ pick_server is ready and waiting for calls...");
    ROS_INFO_STREAM("============================================");

    ros::waitForShutdown();
    return 0;
}