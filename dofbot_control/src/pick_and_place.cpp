/**
 * DOFBOT-Pi MoveIt Pick & Place Demo
 * ROS Noetic | MoveIt 1
 *
 * Planning groups (adjust to match your SRDF):
 *   "arm"     – joints 1-5
 *   "gripper" – finger joint
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <vector>

// ─── Constants ────────────────────────────────────────────────────────────────

static const std::string ARM_GROUP     = "dofbot";
static const std::string GRIPPER_GROUP = "gripper";
static const std::string BASE_FRAME    = "base_link";

// Gripper joint limits (radians) – tune to your SRDF
static const double GRIPPER_OPEN   =  0.0;
static const double GRIPPER_CLOSED =  0.8;

// ─── Helper: build a geometry_msgs::Pose ─────────────────────────────────────

geometry_msgs::Pose makePose(double x, double y, double z,
                              double roll, double pitch, double yaw)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

// ─── Helper: add a box collision object to the scene ─────────────────────────

void addCollisionBox(moveit::planning_interface::PlanningSceneInterface& psi,
                     const std::string& id,
                     double x, double y, double z,
                     double sx, double sy, double sz)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = BASE_FRAME;

  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};

  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::CollisionObject::ADD;

  psi.applyCollisionObject(obj);
  ROS_INFO("Added collision object: %s", id.c_str());
}

// ─── Helper: move arm to a named configuration ───────────────────────────────

bool moveToNamed(moveit::planning_interface::MoveGroupInterface& arm,
                 const std::string& target_name)
{
  arm.setNamedTarget(target_name);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (ok)
  {
    arm.execute(plan);
    ROS_INFO("Moved to named target: %s", target_name.c_str());
  }
  else
    ROS_WARN("Planning failed for named target: %s", target_name.c_str());
  return ok;
}

// ─── Helper: move arm to a pose goal ─────────────────────────────────────────

bool moveToPose(moveit::planning_interface::MoveGroupInterface& arm,
                const geometry_msgs::Pose& target_pose,
                const std::string& label = "goal")
{
  arm.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (ok)
  {
    arm.execute(plan);
    ROS_INFO("Moved to pose: %s", label.c_str());
  }
  else
    ROS_WARN("Planning failed for pose: %s", label.c_str());
  return ok;
}

// ─── Helper: move arm along a Cartesian path ─────────────────────────────────

bool moveCartesian(moveit::planning_interface::MoveGroupInterface& arm,
                   const std::vector<geometry_msgs::Pose>& waypoints,
                   double eef_step = 0.005,
                   double jump_threshold = 0.0)
{
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = arm.computeCartesianPath(waypoints, eef_step,
                                              jump_threshold, trajectory);
  ROS_INFO("Cartesian path coverage: %.1f%%", fraction * 100.0);
  if (fraction < 0.9)
  {
    ROS_WARN("Cartesian path coverage too low, aborting.");
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  arm.execute(plan);
  return true;
}

// ─── Helper: set gripper ─────────────────────────────────────────────────────

bool setGripper(moveit::planning_interface::MoveGroupInterface& gripper,
                double value)
{
  std::vector<double> joint_values = {value};
  gripper.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (gripper.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (ok) gripper.execute(plan);
  else    ROS_WARN("Gripper planning failed.");
  return ok;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dofbot_pick_and_place");
  ros::NodeHandle nh;

  // Async spinner required by MoveIt
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // ── MoveIt interfaces ──────────────────────────────────────────────────────
  moveit::planning_interface::MoveGroupInterface arm(ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper(GRIPPER_GROUP);
  moveit::planning_interface::PlanningSceneInterface psi;

  arm.setPlanningTime(5.0);
  arm.setMaxVelocityScalingFactor(0.4);      // 40% speed – safe for Pi
  arm.setMaxAccelerationScalingFactor(0.3);
  arm.setNumPlanningAttempts(10);

  gripper.setPlanningTime(3.0);
  gripper.setMaxVelocityScalingFactor(0.5);

  ROS_INFO("Reference frame : %s", arm.getPlanningFrame().c_str());
  ROS_INFO("End-effector    : %s", arm.getEndEffectorLink().c_str());

  ros::Duration(1.0).sleep();  // let the scene settle

  // ── 1. Build the planning scene ────────────────────────────────────────────
  //
  //   table  : flat surface the object rests on
  //   object : small cube to be picked
  //   shelf  : destination platform
  //
  //   Coordinates are in metres, relative to base_link.
  //   Adjust to match your physical setup!

  addCollisionBox(psi, "table",   0.15,  0.00, -0.03,  0.40, 0.40, 0.02);
  addCollisionBox(psi, "object",  0.15,  0.00,  0.025, 0.04, 0.04, 0.05);
  addCollisionBox(psi, "shelf",  -0.05,  0.20,  0.04,  0.15, 0.15, 0.08);

  ros::Duration(0.5).sleep();

  // ── 2. Go to home ──────────────────────────────────────────────────────────
  ROS_INFO("=== Step 1: Home ===");
  moveToNamed(arm, "home");           // "home" must be defined in your SRDF
  ros::Duration(0.5).sleep();

  // ── 3. Open gripper ────────────────────────────────────────────────────────
  ROS_INFO("=== Step 2: Open gripper ===");
  setGripper(gripper, GRIPPER_OPEN);
  ros::Duration(0.3).sleep();

  // ── 4. Pre-grasp: approach from above ─────────────────────────────────────
  //   Pointing end-effector straight down (pitch = -π/2)
  ROS_INFO("=== Step 3: Pre-grasp approach ===");
  geometry_msgs::Pose pre_grasp = makePose(0.15, 0.00, 0.12,
                                            0.0, -M_PI_2, 0.0);
  moveToPose(arm, pre_grasp, "pre_grasp");
  ros::Duration(0.3).sleep();

  // ── 5. Descend to grasp via Cartesian path ─────────────────────────────────
  ROS_INFO("=== Step 4: Descend to grasp ===");
  geometry_msgs::Pose grasp_pose = makePose(0.15, 0.00, 0.065,
                                             0.0, -M_PI_2, 0.0);
  std::vector<geometry_msgs::Pose> descend_waypoints = {pre_grasp, grasp_pose};
  moveCartesian(arm, descend_waypoints);
  ros::Duration(0.3).sleep();

  // ── 6. Close gripper & attach object ──────────────────────────────────────
  ROS_INFO("=== Step 5: Grasp ===");
  setGripper(gripper, GRIPPER_CLOSED);
  ros::Duration(0.5).sleep();

  // Attach so MoveIt treats the object as part of the robot
  arm.attachObject("object", arm.getEndEffectorLink());
  ROS_INFO("Object attached to end-effector.");
  ros::Duration(0.3).sleep();

  // ── 7. Lift straight up ────────────────────────────────────────────────────
  ROS_INFO("=== Step 6: Lift ===");
  geometry_msgs::Pose lift_pose = makePose(0.15, 0.00, 0.18,
                                            0.0, -M_PI_2, 0.0);
  std::vector<geometry_msgs::Pose> lift_waypoints = {grasp_pose, lift_pose};
  moveCartesian(arm, lift_waypoints);
  ros::Duration(0.3).sleep();

  // ── 8. Move to pre-place above shelf ──────────────────────────────────────
  ROS_INFO("=== Step 7: Transport ===");
  geometry_msgs::Pose pre_place = makePose(-0.05, 0.20, 0.22,
                                            0.0, -M_PI_2, 0.0);
  moveToPose(arm, pre_place, "pre_place");
  ros::Duration(0.3).sleep();

  // ── 9. Descend to place ────────────────────────────────────────────────────
  ROS_INFO("=== Step 8: Place ===");
  geometry_msgs::Pose place_pose = makePose(-0.05, 0.20, 0.145,
                                             0.0, -M_PI_2, 0.0);
  std::vector<geometry_msgs::Pose> place_waypoints = {pre_place, place_pose};
  moveCartesian(arm, place_waypoints);
  ros::Duration(0.3).sleep();

  // ── 10. Open gripper & detach object ──────────────────────────────────────
  ROS_INFO("=== Step 9: Release ===");
  setGripper(gripper, GRIPPER_OPEN);
  ros::Duration(0.5).sleep();

  arm.detachObject("object");
  ROS_INFO("Object detached.");
  ros::Duration(0.3).sleep();

  // ── 11. Retreat upward ────────────────────────────────────────────────────
  ROS_INFO("=== Step 10: Retreat ===");
  geometry_msgs::Pose retreat_pose = makePose(-0.05, 0.20, 0.22,
                                               0.0, -M_PI_2, 0.0);
  std::vector<geometry_msgs::Pose> retreat_waypoints = {place_pose, retreat_pose};
  moveCartesian(arm, retreat_waypoints);
  ros::Duration(0.3).sleep();

  // ── 12. Return home ────────────────────────────────────────────────────────
  ROS_INFO("=== Step 11: Return home ===");
  moveToNamed(arm, "home");

  ROS_INFO("=== Pick & Place complete! ===");

  ros::shutdown();
  return 0;
}
