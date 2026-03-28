#!/usr/bin/env python3
# coding: utf-8
"""
stacking_grasp_moveit.py  –  ROS1 Noetic + MoveIt1
Fix: remove unsupported approximate=True argument from set_joint_value_target();
     clamp values manually against URDF limits before calling it.
"""

import math
import sys
import rospy
import moveit_commander
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Per-joint calibration  (servo degrees → URDF radians)
# SERVO_ZERO_DEG : servo angle that equals 0 rad in the URDF
# SERVO_DIR      : +1 normal,  -1 if servo and URDF axes are reversed
# ---------------------------------------------------------------------------
JOINT_CALIB = [
    (135.0,  1),   # joint 1 – base
    (135.0,  1),   # joint 2 – shoulder
    (135.0,  1),   # joint 3 – elbow
    (135.0,  1),   # joint 4 – wrist pitch
    (135.0,  1),   # joint 5 – wrist roll
    (135.0,  1),   # joint 6 – gripper (separate group)
]

def servo_to_rad(servo_deg, joint_index):
    zero, direction = JOINT_CALIB[joint_index]
    return math.radians((servo_deg - zero) * direction)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Read joint limits from the MoveIt parameter server
# ---------------------------------------------------------------------------

def get_joint_limits(joint_name):
    """Return (lower, upper) in radians from /robot_description_planning."""
    ns = "/robot_description_planning/joint_limits/{}/".format(joint_name)
    if rospy.get_param(ns + "has_position_limits", False):
        lo = rospy.get_param(ns + "min_position")
        hi = rospy.get_param(ns + "max_position")
        return lo, hi
    # Fall back to joint_limits_interface style
    ns2 = "/robot_description_kinematics/"
    return -math.pi, math.pi   # safe default if nothing found


def clamp_joints_to_limits(group, rads):
    """Clamp a list of joint values to the group's URDF limits."""
    names   = group.get_active_joints()
    clamped = []
    for name, val in zip(names, rads):
        lo, hi = get_joint_limits(name)
        c = clamp(val, lo, hi)
        if abs(c - val) > 1e-4:
            rospy.logwarn("Joint %s: %.4f rad clamped to [%.4f, %.4f] → %.4f",
                          name, val, lo, hi, c)
        clamped.append(c)
    return clamped


# ---------------------------------------------------------------------------
# Robust plan-and-execute  (handles 2-tuple and 4-tuple plan() return values)
# ---------------------------------------------------------------------------

def plan_and_execute(group, retries=3):
    for attempt in range(retries):
        result = group.plan()

        if isinstance(result, tuple):
            if len(result) >= 4:
                success, plan = result[0], result[1]
            elif len(result) == 2:
                # Could be (bool, plan) or (plan, fraction) – detect by type
                success = result[0] if isinstance(result[0], bool) else bool(result[0].joint_trajectory.points)
                plan    = result[1] if isinstance(result[0], bool) else result[0]
            else:
                success, plan = False, None
        else:
            plan    = result
            success = bool(plan and plan.joint_trajectory.points)

        if success and plan and plan.joint_trajectory.points:
            group.execute(plan, wait=True)
            group.stop()
            group.clear_pose_targets()
            return True

        rospy.logwarn("Planning attempt %d/%d failed, retrying…", attempt + 1, retries)
        rospy.sleep(0.3)

    # Last resort – go() replans internally
    rospy.logwarn("plan() failed %d times, falling back to group.go()", retries)
    ok = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return ok


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class StackingGraspMoveIt:

    GRIPPER_OPEN_DEG   = 30.0
    GRIPPER_CLOSED_DEG = 135.0

    def __init__(self):
        rospy.init_node('stacking_grasp_moveit', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot      = moveit_commander.RobotCommander()
        self.scene      = moveit_commander.PlanningSceneInterface()
        self.arm_group  = moveit_commander.MoveGroupCommander("dofbot")
        self.grip_group = moveit_commander.MoveGroupCommander("gripper")

        self.arm_group.set_max_velocity_scaling_factor(0.4)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)

        self.grip_group.set_planning_time(5.0)
        self.grip_group.set_num_planning_attempts(3)

        self.move_status = True
        self.grap_joint  = 135   # gripper closed, servo degrees

        rospy.Subscriber('/arm_command', String, self._command_cb)
        rospy.loginfo("StackingGraspMoveIt ready – listening on /arm_command")

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def _set_arm_joints(self, servo_angles_6dof):
        """Convert servo angles for joints 0-4 → rad, clamp, then plan."""
        arm_rads = [servo_to_rad(servo_angles_6dof[i], i) for i in range(5)]
        arm_rads = clamp_joints_to_limits(self.arm_group, arm_rads)

        try:
            self.arm_group.set_joint_value_target(arm_rads)   # ← no extra arg
        except Exception as exc:
            rospy.logerr("arm set_joint_value_target failed: %s", exc)
            return False

        ok = plan_and_execute(self.arm_group)
        self._set_gripper(servo_angles_6dof[5])
        return ok

    def _set_gripper(self, servo_deg):
        """Map servo degrees to clamped radians, then command gripper group."""
        span  = self.GRIPPER_CLOSED_DEG - self.GRIPPER_OPEN_DEG
        ratio = clamp((servo_deg - self.GRIPPER_OPEN_DEG) / span, 0.0, 1.0)
        rad   = ratio * (math.pi / 2.0)

        # Clamp to actual URDF limits
        grip_joints = self.grip_group.get_active_joints()
        if grip_joints:
            lo, hi = get_joint_limits(grip_joints[0])
            rad = clamp(rad, lo, hi)

        try:
            self.grip_group.set_joint_value_target([rad])      # ← no extra arg
        except Exception as exc:
            rospy.logerr("gripper set_joint_value_target failed: %s", exc)
            return False

        return plan_and_execute(self.grip_group)

    def _wiggle_gripper(self, cycles=5):
        for _ in range(cycles):
            self._set_gripper(180)
            rospy.sleep(0.08)
            self._set_gripper(30)
            rospy.sleep(0.08)

    # ------------------------------------------------------------------
    # Core motion sequence
    # ------------------------------------------------------------------

    def move(self, joints, joints_down):
        joints_00 = [90,  80, 50, 50, 265, self.grap_joint]
        joints_up = [135, 80, 50, 50, 265, 30]

        self._set_arm_joints(joints_00);            rospy.sleep(1.0)
        self._wiggle_gripper(cycles=5)
        self._set_arm_joints(joints);               rospy.sleep(1.0)
        self._set_gripper(self.grap_joint);         rospy.sleep(0.5)
        self._set_arm_joints(joints_00);            rospy.sleep(1.0)

        mid = [joints_down[0], 80, 50, 50, 265, self.grap_joint]
        self._set_arm_joints(mid);                  rospy.sleep(1.0)
        self._set_arm_joints(joints_down);          rospy.sleep(1.5)
        self._set_gripper(30);                      rospy.sleep(0.5)
        self._set_arm_joints(joints_up);            rospy.sleep(1.0)

    # ------------------------------------------------------------------
    # Stack-level dispatcher
    # ------------------------------------------------------------------

    def arm_run(self, move_num, joints):
        place_poses = {
            '1': [135, 50, 20, 60, 265, self.grap_joint],
            '2': [135, 55, 38, 38, 265, self.grap_joint],
            '3': [135, 60, 45, 30, 265, self.grap_joint],
            '4': [135, 65, 55, 20, 265, self.grap_joint],
        }
        if move_num not in place_poses:
            rospy.logwarn("Unknown move_num: %s", move_num)
            return
        if not self.move_status:
            rospy.logwarn("Arm busy – ignoring command %s", move_num)
            return

        self.move_status = False
        pick = [joints[0], joints[1], joints[2], joints[3], 265, 30]
        self.move(pick, place_poses[move_num])
        self.move_status = True

    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------

    def _command_cb(self, msg):
        try:
            parts    = msg.data.strip().split(',')
            move_num = parts[0]
            joints   = [float(p) for p in parts[1:5]]
            rospy.loginfo("Command: level=%s joints=%s", move_num, joints)
            self.arm_run(move_num, joints)
        except (IndexError, ValueError) as exc:
            rospy.logerr("Bad /arm_command '%s': %s", msg.data, exc)

    def run(self):
        rospy.spin()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = StackingGraspMoveIt()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()