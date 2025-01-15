#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <mutex>
#include <cmath>
#include <functional>
#include <iostream>

// Constants
const double TAU = 2 * M_PI;
const double POSE_PRECISION = 1e-3; // Tolerance for comparing poses

// Helper to create a box collision object
moveit_msgs::CollisionObject createBox(const std::string &id,
                                       const geometry_msgs::Pose &pose,
                                       const std::vector<double> &dims,
                                       const std::string &frame_id = "base_link")
{
    moveit_msgs::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = frame_id;
    obj.operation = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive box_prim;
    box_prim.type = shape_msgs::SolidPrimitive::BOX;
    box_prim.dimensions = dims; // {x_size, y_size, z_size}

    obj.primitives.push_back(box_prim);
    obj.primitive_poses.push_back(pose);

    return obj;
}

// Create your collision objects (e.g., floor, walls, test box)
std::vector<moveit_msgs::CollisionObject> createCollisionObjects()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.reserve(6);

    // 1) Box under the robot
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z = -0.05;
        collision_objects.push_back(createBox("box_under_robot", pose, {0.81, 0.74, 0.1}));
    }

    // 2) Front wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.y = 0.37;
        pose.position.z = 0.59 / 2.0; // wall_height / 2.0
        collision_objects.push_back(createBox("front_wall", pose, {0.81, 0.02, 0.59}));
    }

    // 3) Back wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.y = -0.37;
        pose.position.z = 0.59 / 2.0;
        collision_objects.push_back(createBox("back_wall", pose, {0.81, 0.02, 0.59}));
    }

    // 4) Left wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = -0.40;
        pose.position.z = 0.59 / 2.0;
        collision_objects.push_back(createBox("left_wall", pose, {0.02, 0.74, 0.59}));
    }

    // 5) Right wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.40;
        pose.position.z = 0.59 / 2.0;
        collision_objects.push_back(createBox("right_wall", pose, {0.02, 0.74, 0.59}));
    }
    
    // // 6) New box at the arrow's location
    //  {
    //      geometry_msgs::Pose pose;
    //      pose.orientation.w = 1.0;
    //      pose.position.x = 0.3; // Replace with the actual X coordinate of the arrow
    //      pose.position.y = -0.1; // Replace with the actual Y coordinate of the arrow
    //      pose.position.z = 0.075; // Half of the height (15 cm / 2) to center the box on the floor
    //      collision_objects.push_back(createBox("small_box", pose, {0.15, 0.15, 0.15})); // 15 cm x 15 cm x 15 cm box
    //  }

    return collision_objects;
}

// Synchronous planning function
bool performSynchronousPlanning(moveit::planning_interface::MoveGroupInterface &group,
                                moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                int max_retries = 10)
{
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        moveit::planning_interface::MoveItErrorCode result = group.plan(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Planning succeeded on attempt %d.", attempt);
            return true;
        } else {
            ROS_WARN("Planning attempt %d failed with error code: %d.", attempt, result.val);
        }
    }
    ROS_ERROR("Motion planning failed after %d attempts.", max_retries);
    return false;
}

// Function to ensure strictly increasing time_from_start
bool fixTrajectoryTiming(moveit_msgs::RobotTrajectory &trajectory, double min_increment = 0.1)
{
    bool fixed = false;
    for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
        if (trajectory.joint_trajectory.points[i].time_from_start <= trajectory.joint_trajectory.points[i - 1].time_from_start) {
            trajectory.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i - 1].time_from_start + ros::Duration(min_increment);
            fixed = true;
            ROS_WARN("Adjusted time_from_start for point %zu to %.3f seconds.", i, trajectory.joint_trajectory.points[i].time_from_start.toSec());
        }
    }
    return fixed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_no_wait_code_fixed_with_constraints_no_cache");
    ros::NodeHandle nh;

    if (!ros::master::check()) {
        ROS_ERROR("ROS Master is not running. Exiting...");
        return 1;
    }

    // Start AsyncSpinner
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1.0).sleep();

    // MoveIt: group + PlanningSceneInterface
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Further lowered velocity and acceleration scaling factors
    group.setMaxVelocityScalingFactor(0.75);
    group.setMaxAccelerationScalingFactor(1.0);

    group.setPlannerId("PRMstar");
    group.setNumPlanningAttempts(3);
    double planning_time = 1.0;
    group.setPlanningTime(planning_time);

    // If your robot's root link is different, replace "base_link"
    group.setPoseReferenceFrame("base_link");
    ROS_INFO_STREAM("Planner ID: " << group.getPlannerId());
    ROS_INFO_STREAM("Pose reference frame: " << group.getPoseReferenceFrame());

    // Publisher for trajectory visualization
    ros::Publisher display_pub =
        nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Create a PlanningSceneMonitor (no waiting on /get_planning_scene)
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
    psm.startSceneMonitor("/move_group/monitored_planning_scene");
    psm.startWorldGeometryMonitor();
    psm.startStateMonitor();
    psm.providePlanningSceneService();
    psm.setPlanningScenePublishingFrequency(10.0);

    // 1) Remove old objects
    std::vector<std::string> old_objects = planning_scene_interface.getKnownObjectNames();
    if (!old_objects.empty()) {
        ROS_INFO("Removing %zu old objects from the scene...", old_objects.size());
        planning_scene_interface.removeCollisionObjects(old_objects);
        ros::Duration(1.0).sleep();
    }

    // 2) Create and apply new collision objects
    auto collision_objects = createCollisionObjects();
    ROS_INFO("Applying %zu collision objects (no infinite wait)...", collision_objects.size());

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep(); // Let the scene update briefly

    // Confirm known objects
    auto known_objs = planning_scene_interface.getKnownObjectNames();
    ROS_INFO("Known objects now in the scene:");
    for (const auto &name : known_objs) {
        ROS_INFO_STREAM("  - " << name);
    }

    // 3) Define target poses with the same orientation
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-TAU / 2, 0, 0); // Example orientation (straight)
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.276;
    target_pose1.position.y = -0.197;
    target_pose1.position.z = 0.250;

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.239;
    target_pose2.position.y = 0.112;
    target_pose2.position.z = 0.125;

    // *** Reintroduce Orientation Path Constraints ***
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose1.orientation;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 0.1;

    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(path_constraints);
    ROS_INFO("Orientation path constraints set to maintain EEF straight.");

    // Variables for dynamic planning time
    int consecutive_failures = 0;
    const int max_failures = 5;
    const double planning_time_increment = 1.0;
    const double planning_time_decrement = 0.5;
    const double min_planning_time = 2.0;
    const double max_planning_time = 10.0;

    // 4) Main planning & execution loop
    bool toggle = true;
    ROS_INFO("Starting planning and execution loop with orientation constraints (no caching)...");
    while (ros::ok()) {
        // Current robot pose
        geometry_msgs::PoseStamped current_pose_stamped = group.getCurrentPose();
        geometry_msgs::Pose current_pose = current_pose_stamped.pose;

        // Toggle target pose
        geometry_msgs::Pose target_pose = (toggle ? target_pose1 : target_pose2);

        moveit::planning_interface::MoveGroupInterface::Plan global_plan;

        // Set the new goal pose
        group.setStartStateToCurrentState(); // Ensure start state is current
        group.setPoseTarget(target_pose);

        // Time the planning
        ros::Time start_time = ros::Time::now();

        bool planning_success = performSynchronousPlanning(group, global_plan, 10);

        if (!planning_success) {
            // Increase planning time
            planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
            group.setPlanningTime(planning_time);
            consecutive_failures++;

            if (consecutive_failures >= max_failures) {
                ROS_WARN("Max consecutive failures. Reset planning time to 5.0s");
                planning_time = 5.0;
                group.setPlanningTime(planning_time);
                consecutive_failures = 0;
            }

            group.clearPoseTargets();
            ros::Duration(0.5).sleep();
            toggle = !toggle;
            continue;
        }

        ros::Duration plan_dur = ros::Time::now() - start_time;
        ROS_INFO("Planning took %.3f seconds", plan_dur.toSec());

        // Check if plan is valid (>= 2 points)
        auto &traj = global_plan.trajectory_.joint_trajectory;
        if (traj.points.size() < 2) {
            ROS_ERROR("Plan has fewer than 2 trajectory points. Discarding and retrying...");
            // Increase planning time
            planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
            group.setPlanningTime(planning_time);
            consecutive_failures++;

            if (consecutive_failures >= max_failures) {
                ROS_WARN("Max consecutive failures. Reset planning time to 5.0s");
                planning_time = 5.0;
                group.setPlanningTime(planning_time);
                consecutive_failures = 0;
            }

            group.clearPoseTargets();
            ros::Duration(0.5).sleep();
            toggle = !toggle;
            continue;
        }

        // Verify and fix strictly increasing time_from_start
        bool timing_fixed = fixTrajectoryTiming(global_plan.trajectory_);
        if (timing_fixed) {
            ROS_WARN("Fixed trajectory timing to ensure strictly increasing time_from_start.");
        }

        // Re-verify after fixing
        bool time_ok = true;
        const auto &points = traj.points;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].time_from_start <= points[i - 1].time_from_start) {
                ROS_ERROR("Trajectory message not strictly increasing in time at index %zu!", i);
                time_ok = false;
                break;
            }
        }

        if (!time_ok) {
            ROS_ERROR("Discarding this plan due to invalid timing even after fixing. Retrying...");
            // Increase planning time
            planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
            group.setPlanningTime(planning_time);
            consecutive_failures++;

            if (consecutive_failures >= max_failures) {
                ROS_WARN("Max consecutive failures. Reset planning time to 5.0s");
                planning_time = 5.0;
                group.setPlanningTime(planning_time);
                consecutive_failures = 0;
            }

            group.clearPoseTargets();
            ros::Duration(0.5).sleep();
            toggle = !toggle;
            continue;
        }

        // Visualize the trajectory
        display_trajectory.trajectory_start = global_plan.start_state_;
        display_trajectory.trajectory.clear();
        display_trajectory.trajectory.push_back(global_plan.trajectory_);
        display_pub.publish(display_trajectory);

        ROS_INFO("Plan succeeded. Executing now...");

        // Execute the plan
        moveit::core::MoveItErrorCode exec_result = group.execute(global_plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Execution succeeded. Target reached.");
            // Decrease planning time
            planning_time = std::max(min_planning_time, planning_time - planning_time_decrement);
            group.setPlanningTime(planning_time);
            consecutive_failures = 0;
        } else {
            ROS_ERROR("Execution failed. Retrying...");
            // Increase planning time
            planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
            group.setPlanningTime(planning_time);
        }

        group.clearPoseTargets();
        ros::Duration(0.5).sleep();
        toggle = !toggle;
    }

    // Cleanup
    group.clearPathConstraints();
    ros::shutdown();
    return 0;
}
