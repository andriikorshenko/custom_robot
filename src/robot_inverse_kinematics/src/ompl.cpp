/****************************************************
 * Final Example Without Waiting for /get_planning_scene
 ****************************************************/

// Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <future>
#include <unordered_map>
#include <ros/ros.h>
#include <mutex>
#include <cmath>
#include <functional>
#include <iostream>

// Constants
const double TAU = 2 * M_PI;
const double POSE_PRECISION = 1e-3; // Tolerance for comparing poses

// Mutex for thread-safe operations
std::mutex plan_cache_mutex;

// Structure to hold a pair of poses (start and goal)
struct PosePair {
    geometry_msgs::Pose start;
    geometry_msgs::Pose goal;

    bool operator==(const PosePair &other) const {
        auto eq = [&](const geometry_msgs::Pose &a, const geometry_msgs::Pose &b) {
            return (std::fabs(a.position.x - b.position.x) < POSE_PRECISION &&
                    std::fabs(a.position.y - b.position.y) < POSE_PRECISION &&
                    std::fabs(a.position.z - b.position.z) < POSE_PRECISION &&
                    std::fabs(a.orientation.x - b.orientation.x) < POSE_PRECISION &&
                    std::fabs(a.orientation.y - b.orientation.y) < POSE_PRECISION &&
                    std::fabs(a.orientation.z - b.orientation.z) < POSE_PRECISION &&
                    std::fabs(a.orientation.w - b.orientation.w) < POSE_PRECISION);
        };
        return eq(start, other.start) && eq(goal, other.goal);
    }
};

// Custom hash function for PosePair
struct PosePairHash {
    std::size_t operator()(const PosePair& p) const {
        auto hash_double = std::hash<double>();
        auto quantize = [&](double v) -> double {
            return std::round(v / POSE_PRECISION);
        };
        auto hash_pose = [&](const geometry_msgs::Pose &pose) -> std::size_t {
            std::size_t seed = 0;
            // Position
            seed ^= hash_double(quantize(pose.position.x)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.position.y)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.position.z)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

            // Orientation
            seed ^= hash_double(quantize(pose.orientation.x)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.orientation.y)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.orientation.z)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.orientation.w)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

            return seed;
        };

        std::size_t seed = 0;
        seed ^= hash_pose(p.start) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hash_pose(p.goal) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

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

    // 2) Walls
    double wall_height = 0.59;    // 590mm
    double wall_length = 0.81;    // 810mm
    double wall_thickness = 0.02; // 20mm

    // Front wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.y = 0.37;
        pose.position.z = wall_height / 2.0;
        collision_objects.push_back(createBox("front_wall", pose, {wall_length, wall_thickness, wall_height}));
    }

    // Back wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.y = -0.37;
        pose.position.z = wall_height / 2.0;
        collision_objects.push_back(createBox("back_wall", pose, {wall_length, wall_thickness, wall_height}));
    }

    // Left wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = -0.40;
        pose.position.z = wall_height / 2.0;
        collision_objects.push_back(createBox("left_wall", pose, {wall_thickness, 0.74, wall_height}));
    }

    // Right wall
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.40;
        pose.position.z = wall_height / 2.0;
        collision_objects.push_back(createBox("right_wall", pose, {wall_thickness, 0.74, wall_height}));
    }

    return collision_objects;
}

// Asynchronous planning function
moveit::planning_interface::MoveGroupInterface::Plan performAsynchronousPlanning(
    moveit::planning_interface::MoveGroupInterface &group, int max_retries = 10)
{
    return std::async(std::launch::async, [&group, max_retries]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            auto result = group.plan(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                ROS_INFO("Planning succeeded on attempt %d.", attempt);
                return plan;
            } else {
                ROS_WARN("Planning attempt %d failed with error code: %d.", attempt, result.val);
            }
        }
        throw std::runtime_error("Motion planning failed after multiple attempts.");
    }).get();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_no_wait_code");
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

    group.setMaxVelocityScalingFactor(0.75);
    group.setMaxAccelerationScalingFactor(1.0);
    group.setPlannerId("RRTConnect");
    double planning_time = 2.0;
    group.setPlanningTime(planning_time);

    // If your robot's root link is different, replace "base_link"
    group.setPoseReferenceFrame("base_link");
    ROS_INFO_STREAM("Planner ID: " << group.getPlannerId());
    ROS_INFO_STREAM("Pose reference frame: " << group.getPoseReferenceFrame());

    // Publisher for trajectory visualization
    ros::Publisher display_pub =
        nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Create a PlanningSceneMonitor but DO NOT call requestPlanningSceneState()
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
    // We start monitors so we see updates, but no waiting on /get_planning_scene:
    psm.startSceneMonitor("/move_group/monitored_planning_scene");
    psm.startWorldGeometryMonitor();
    psm.startStateMonitor();
    psm.providePlanningSceneService();
    psm.setPlanningScenePublishingFrequency(10.0);

    // 1) Remove old objects (no infinite wait).
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

    // 3) Define target poses and constraints
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-TAU / 2, 0, 0); // Example orientation
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.301;
    target_pose1.position.y = -0.303;
    target_pose1.position.z = 0.121;

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.313;
    target_pose2.position.y = -0.231;
    target_pose2.position.z = 0.035;

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose1.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(path_constraints);

    // 4) Plan cache
    std::unordered_map<PosePair, moveit::planning_interface::MoveGroupInterface::Plan, PosePairHash> plan_cache;

    // Variables for dynamic planning time
    int consecutive_failures = 0;
    const int max_failures = 5;
    const double planning_time_increment = 1.0;
    const double planning_time_decrement = 0.5;
    const double min_planning_time = 2.0;
    const double max_planning_time = 10.0;

    // 5) Main planning & execution loop
    bool toggle = true;
    ROS_INFO("Starting planning and execution loop...");
    while (ros::ok()) {
        // Current robot pose
        geometry_msgs::PoseStamped current_pose_stamped = group.getCurrentPose();
        geometry_msgs::Pose current_pose = current_pose_stamped.pose;

        // Toggle target pose
        geometry_msgs::Pose target_pose = toggle ? target_pose1 : target_pose2;

        // Create a PosePair
        PosePair current_pair{current_pose, target_pose};

        moveit::planning_interface::MoveGroupInterface::Plan global_plan;
        bool plan_found_in_cache = false;

        // Check plan cache
        {
            std::lock_guard<std::mutex> lock(plan_cache_mutex);
            auto it = plan_cache.find(current_pair);
            if (it != plan_cache.end()) {
                ROS_INFO("Found plan in cache. Reusing cached trajectory.");
                global_plan = it->second;
                plan_found_in_cache = true;
            }
        }

        if (!plan_found_in_cache) {
            // Not cached -> plan
            group.setPoseTarget(target_pose);
            ros::Time start_time = ros::Time::now();

            try {
                global_plan = performAsynchronousPlanning(group, 10);
            } catch (const std::exception &e) {
                ROS_ERROR("Asynchronous planning failed: %s", e.what());
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

            // Validate plan
            if (!global_plan.trajectory_.joint_trajectory.points.empty()) {
                // Cache new plan
                {
                    std::lock_guard<std::mutex> lock(plan_cache_mutex);
                    plan_cache[current_pair] = global_plan;
                }
                ROS_INFO("Plan succeeded and cached. Executing now...");

                // Visualize
                display_trajectory.trajectory_start = global_plan.start_state_;
                display_trajectory.trajectory.clear();
                display_trajectory.trajectory.push_back(global_plan.trajectory_);
                display_pub.publish(display_trajectory);
            } else {
                ROS_ERROR("Planning produced an empty trajectory. Retrying...");

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
        }

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
