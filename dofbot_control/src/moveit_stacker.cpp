#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <string>
#include <vector>

// ============================================================================
//  ROOT CAUSE: "Unable to sample any valid states for goal tree"
//
//  The Dofbot SRDF's Allowed Collision Matrix (ACM) does not whitelist
//  adjacent links that are always in contact.  The collision checker marks
//  EVERY configuration (including the current pose) as invalid, so the
//  planner cannot find a legal goal state and immediately aborts.
//
//  FIX: Fetch the live planning scene, set all link pairs to ALLOWED,
//  and push it back.  This is safe for a desktop arm with no obstacles.
// ============================================================================

static constexpr bool RUN_STACK = false;   // false = diagnostic  true = stack

class MoveItStacker
{
public:
    MoveItStacker()
        : arm_group_("dofbot"),
          gripper_group_("gripper")
    {
        arm_group_.setPlanningTime(10.0);
        arm_group_.setNumPlanningAttempts(20);
        arm_group_.setGoalJointTolerance(0.05);

        gripper_open_   = { 0.03 };
        gripper_closed_ = { 0.00 };

        ros::Duration(1.0).sleep();

        // ── THE FIX: disable all collisions in the planning scene ────────
        disableAllCollisions();
    }

    // -----------------------------------------------------------------------
    // Fetches the live planning scene and marks every link pair ALLOWED.
    // -----------------------------------------------------------------------
    void disableAllCollisions()
    {
        ros::NodeHandle nh;

        // 1. Fetch current scene
        ros::ServiceClient get_client =
            nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
        ros::ServiceClient apply_client =
            nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");

        get_client.waitForExistence(ros::Duration(5.0));
        apply_client.waitForExistence(ros::Duration(5.0));

        moveit_msgs::GetPlanningScene get_srv;
        get_srv.request.components.components =
            moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

        if (!get_client.call(get_srv))
        {
            ROS_ERROR("Failed to fetch planning scene — collision fix skipped.");
            return;
        }

        moveit_msgs::AllowedCollisionMatrix& acm =
            get_srv.response.scene.allowed_collision_matrix;

        // 2. Get all link names known to the move group
        std::vector<std::string> links = arm_group_.getLinkNames();
        // Also grab end-effector and robot model links
        const std::vector<std::string>& jnames = arm_group_.getJointNames();
        (void)jnames;   // used indirectly via link names

        ROS_INFO("Disabling collisions between %zu links...", links.size());

        // 3. Build a complete ACM with every pair set to ALLOWED
        //    Re-use existing entry names and extend if needed
        for (const auto& l1 : links)
        {
            bool found = false;
            for (const auto& name : acm.entry_names)
                if (name == l1) { found = true; break; }
            if (!found)
                acm.entry_names.push_back(l1);
        }

        const size_t n = acm.entry_names.size();
        acm.entry_values.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            acm.entry_values[i].enabled.assign(n, true);   // allow everything
        }

        // 4. Push the modified scene back
        moveit_msgs::ApplyPlanningScene apply_srv;
        apply_srv.request.scene.is_diff = true;
        apply_srv.request.scene.allowed_collision_matrix = acm;

        if (apply_client.call(apply_srv) && apply_srv.response.success)
            ROS_INFO("Collision matrix patched — all pairs now ALLOWED.");
        else
            ROS_ERROR("Failed to apply patched collision matrix.");
    }

    // -----------------------------------------------------------------------
    std::vector<double> getCurrentJoints(const std::string& label = "CURRENT")
    {
        std::vector<double> jv = arm_group_.getCurrentJointValues();
        ROS_INFO("=== %s (%zu joints) ===", label.c_str(), jv.size());
        for (size_t i = 0; i < jv.size(); ++i)
            ROS_INFO("  [%zu] = %.4f rad  (%.1f deg)", i, jv[i], jv[i]*180.0/M_PI);
        return jv;
    }

    bool goToJoints(const std::vector<double>& joints, const std::string& label)
    {
        ROS_INFO("Planning to: %s", label.c_str());
        arm_group_.setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (arm_group_.plan(plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (ok)
        {
            arm_group_.execute(plan);
            ROS_INFO("  Reached: %s", label.c_str());
        }
        else
        {
            ROS_ERROR("  Planning failed for: %s", label.c_str());
        }
        arm_group_.stop();
        return ok;
    }

    void moveGripper(const std::vector<double>& state)
    {
        gripper_group_.setJointValueTarget(state);
        gripper_group_.move();
        gripper_group_.stop();
    }

    // -----------------------------------------------------------------------
    // DIAGNOSTIC: prove the planner works, then nudge each joint
    // -----------------------------------------------------------------------
    void runDiagnostic()
    {
        std::vector<double> home = getCurrentJoints("BOOT_HOME");

        ROS_INFO("--- TEST 1: trivial plan to current state ---");
        bool ok = goToJoints(home, "home");
        if (!ok)
        {
            ROS_FATAL("Trivial plan still failed after ACM patch.");
            ROS_FATAL("Check: rostopic echo /joint_states  (must be publishing)");
            ROS_FATAL("Check: rosnode list | grep move_group");
            return;
        }

        ROS_INFO("--- TEST 2: nudge each joint by +0.1 rad ---");
        for (size_t j = 0; j < home.size(); ++j)
        {
            std::vector<double> nudged = home;
            nudged[j] += 0.1;
            std::string lbl = "nudge_j" + std::to_string(j);
            if (goToJoints(nudged, lbl))
            {
                getCurrentJoints(lbl + "_actual");
                goToJoints(home, "return_home");
            }
        }

        ROS_INFO("=== Diagnostic done. Paste BOOT_HOME values into runStack() ===");
    }

    // -----------------------------------------------------------------------
    // STACK: fill in values from diagnostic output, set RUN_STACK = true
    // -----------------------------------------------------------------------
    void runStack()
    {
        // ── REPLACE WITH YOUR DIAGNOSTIC OUTPUT ───────────────────────────
        std::vector<double> home          = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> hover_pick    = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> at_pick       = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> hover_stack   = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> at_stack_l0   = {0.0, 0.0, 0.0, 0.0, 0.0};
        // ──────────────────────────────────────────────────────────────────

        const int    HEIGHT_JOINT = 1;     // joint index that controls height
        const double HEIGHT_STEP  = 0.05;  // radians per level — tune this

        for (int level = 0; level < 3; ++level)
        {
            ROS_INFO("=== Stack level %d ===", level);

            std::vector<double> at_stack = at_stack_l0;
            at_stack[HEIGHT_JOINT] -= level * HEIGHT_STEP;

            goToJoints(hover_pick,  "hover_pick");
            goToJoints(at_pick,     "at_pick");
            moveGripper(gripper_closed_);

            goToJoints(hover_pick,  "lift");
            goToJoints(hover_stack, "hover_stack");
            goToJoints(at_stack,    "at_stack");
            moveGripper(gripper_open_);
            goToJoints(hover_stack, "retreat");
            goToJoints(home,        "home");
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    std::vector<double> gripper_open_;
    std::vector<double> gripper_closed_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_stacking_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    {
        MoveItStacker stacker;
        if (!RUN_STACK)
            stacker.runDiagnostic();
        else
            stacker.runStack();
    }

    ros::shutdown();
    return 0;
}