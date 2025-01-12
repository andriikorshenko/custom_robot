// Include necessary headers
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
const double POSE_PRECISION = 1e-3; // Precision for pose quantization

// Mutex for thread-safe operations
std::mutex plan_cache_mutex;

// Structure to hold a pair of poses (start and goal)
struct PosePair {
    geometry_msgs::Pose start;
    geometry_msgs::Pose goal;

    bool operator==(const PosePair &other) const {
        // Compare poses with some tolerance to account for floating point precision
        auto poses_are_equal = [](const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) -> bool {
            return std::abs(a.position.x - b.position.x) < POSE_PRECISION &&
                   std::abs(a.position.y - b.position.y) < POSE_PRECISION &&
                   std::abs(a.position.z - b.position.z) < POSE_PRECISION &&
                   std::abs(a.orientation.x - b.orientation.x) < POSE_PRECISION &&
                   std::abs(a.orientation.y - b.orientation.y) < POSE_PRECISION &&
                   std::abs(a.orientation.z - b.orientation.z) < POSE_PRECISION &&
                   std::abs(a.orientation.w - b.orientation.w) < POSE_PRECISION;
        };

        return poses_are_equal(start, other.start) && poses_are_equal(goal, other.goal);
    }
};

// Custom hash function for PosePair
struct PosePairHash {
    std::size_t operator()(const PosePair& p) const {
        auto hash_pose = [&](const geometry_msgs::Pose& pose) -> std::size_t {
            std::size_t seed = 0;
            auto hash_double = std::hash<double>();

            // Quantize the pose values based on POSE_PRECISION
            auto quantize = [&](double value) -> double {
                return std::round(value / POSE_PRECISION);
            };

            seed ^= hash_double(quantize(pose.position.x)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.position.y)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_double(quantize(pose.position.z)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

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

// Function to set up collision objects
std::vector<moveit_msgs::CollisionObject> createCollisionObjects() {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.reserve(5); // Adjusted to the number of objects created

    // Helper lambda to create a box
    auto createBox = [&](const std::string& id, const geometry_msgs::Pose& pose, const std::vector<double>& dimensions) -> moveit_msgs::CollisionObject {
        moveit_msgs::CollisionObject box;
        box.header.frame_id = "base_link";
        box.id = id;

        shape_msgs::SolidPrimitive box_primitive;
        box_primitive.type = shape_msgs::SolidPrimitive::BOX;
        box_primitive.dimensions = dimensions;

        box.primitives.push_back(box_primitive);
        box.primitive_poses.push_back(pose);
        box.operation = moveit_msgs::CollisionObject::ADD;

        return box;
    };

    // Box under the robot
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05;
    collision_objects.push_back(createBox("box", box_pose, {0.81, 0.74, 0.1}));

    // Define wall parameters
    double wall_height = 0.59; // 590mm
    double wall_length = 0.81; // 810mm
    double wall_thickness = 0.02; // 20mm

    // Front wall
    geometry_msgs::Pose front_wall_pose;
    front_wall_pose.orientation.w = 1.0;
    front_wall_pose.position.x = 0.0;
    front_wall_pose.position.y = 0.37; // Positioned in front of the box
    front_wall_pose.position.z = wall_height / 2.0; // Centered
    collision_objects.push_back(createBox("front_wall", front_wall_pose, {wall_length, wall_thickness, wall_height}));

    // Back wall
    geometry_msgs::Pose back_wall_pose;
    back_wall_pose.orientation.w = 1.0;
    back_wall_pose.position.x = 0.0;
    back_wall_pose.position.y = -0.37; // Positioned behind the box
    back_wall_pose.position.z = wall_height / 2.0; // Centered
    collision_objects.push_back(createBox("back_wall", back_wall_pose, {wall_length, wall_thickness, wall_height}));

    // Left wall
    geometry_msgs::Pose left_wall_pose;
    left_wall_pose.orientation.w = 1.0;
    left_wall_pose.position.x = -0.40; // Left side
    left_wall_pose.position.y = 0.0;    // Centered
    left_wall_pose.position.z = wall_height / 2.0; // Centered
    collision_objects.push_back(createBox("left_wall", left_wall_pose, {wall_thickness, 0.74, wall_height}));

    // Right wall
    geometry_msgs::Pose right_wall_pose;
    right_wall_pose.orientation.w = 1.0;
    right_wall_pose.position.x = 0.40; // Right side
    right_wall_pose.position.y = 0.0;    // Centered
    right_wall_pose.position.z = wall_height / 2.0; // Centered
    collision_objects.push_back(createBox("right_wall", right_wall_pose, {wall_thickness, 0.74, wall_height}));

    return collision_objects;
}

// Function to perform asynchronous planning with retries
moveit::planning_interface::MoveGroupInterface::Plan performAsynchronousPlanning(moveit::planning_interface::MoveGroupInterface& group, int max_retries = 10) {
    return std::async(std::launch::async, [&group, max_retries]() -> moveit::planning_interface::MoveGroupInterface::Plan {
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
    }).get(); // Wait and get the result
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_planner_with_caching");
    ros::NodeHandle nh;

    if (!ros::master::check()) {
        ROS_ERROR("ROS Master is not running. Exiting...");
        return -1;
    }

    // Initialize AsyncSpinner with 2 threads
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1.0).sleep(); // Allow some time for the spinner to start

    // Initialize MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Set planning parameters
    group.setMaxVelocityScalingFactor(0.75);
    group.setMaxAccelerationScalingFactor(1);
    group.setPlannerId("RRTConnect");
    double planning_time = 2.0;
    group.setPlanningTime(planning_time);
    group.setPoseReferenceFrame("base_link");

    // Initialize publisher for trajectory visualization
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Monitor the planning scene
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
    if (!psm.requestPlanningSceneState()) {
        ROS_WARN("Failed to request the planning scene state. Proceeding anyway.");
    }

    // Add collision objects to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects = createCollisionObjects();
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("Added collision objects. Waiting for the planning scene to update...");
    ros::Duration(1.0).sleep(); // Allow time for collision objects to be added

    // Define target poses
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-TAU / 2, 0, 0); 
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.301;
    target_pose1.position.y = -0.303;
    target_pose1.position.z = 0.121;

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.313;
    target_pose2.position.y = -0.231;
    target_pose2.position.z = 0.035;

    // Define orientation constraints
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

    // Initialize plan cache
    std::unordered_map<PosePair, moveit::planning_interface::MoveGroupInterface::Plan, PosePairHash> plan_cache;

    // Variables for dynamic planning time adjustment
    int consecutive_failures = 0;
    const int max_failures = 5;
    const double planning_time_increment = 1.0;
    const double planning_time_decrement = 0.5;
    const double min_planning_time = 2.0;
    const double max_planning_time = 10.0;

    // Planning and execution loop with asynchronous planning and caching
    bool toggle = true;
    ROS_INFO("Starting planning and execution loop...");
    while (ros::ok())
    {
        // Get current start pose
        geometry_msgs::PoseStamped current_pose_stamped = group.getCurrentPose();
        geometry_msgs::Pose current_pose = current_pose_stamped.pose;

        // Define target pose based on toggle
        geometry_msgs::Pose target_pose = toggle ? target_pose1 : target_pose2;

        // Create a PosePair
        PosePair current_pair;
        current_pair.start = current_pose;
        current_pair.goal = target_pose;

        // Attempt to retrieve plan from cache
        moveit::planning_interface::MoveGroupInterface::Plan global_plan;
        bool plan_found_in_cache = false;

        {
            std::lock_guard<std::mutex> lock(plan_cache_mutex);
            auto it = plan_cache.find(current_pair);
            if (it != plan_cache.end()) {
                ROS_INFO("Found plan in cache. Reusing the cached trajectory.");
                global_plan = it->second;
                plan_found_in_cache = true;
            }
        }

        if (!plan_found_in_cache) {
            // If not found in cache, perform planning

            // Set the target pose
            group.setPoseTarget(target_pose);

            // Start timing for profiling
            ros::Time start_time = ros::Time::now();

            try {
                // Perform asynchronous planning with retries
                global_plan = performAsynchronousPlanning(group, 10);
            } catch (const std::exception& e) {
                ROS_ERROR("Asynchronous planning failed: %s", e.what());

                // Adjust planning time
                planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
                group.setPlanningTime(planning_time);
                consecutive_failures++;

                if (consecutive_failures >= max_failures) {
                    ROS_WARN("Maximum consecutive planning failures reached. Resetting planning time.");
                    planning_time = 5.0; // Reset to default
                    group.setPlanningTime(planning_time);
                    consecutive_failures = 0;
                }

                // Clear pose targets and continue
                group.clearPoseTargets();
                ros::Duration(0.5).sleep();
                toggle = !toggle;
                continue;
            }

            // Calculate planning duration
            ros::Duration planning_duration = ros::Time::now() - start_time;
            ROS_INFO("Planning took %.3f seconds", planning_duration.toSec());

            // Check if the plan is valid
            if (!global_plan.trajectory_.joint_trajectory.points.empty()) {
                // Cache the new plan
                {
                    std::lock_guard<std::mutex> lock(plan_cache_mutex);
                    plan_cache[current_pair] = global_plan;
                }

                ROS_INFO("Plan succeeded and cached. Executing trajectory...");

                // Publish the trajectory for visualization
                display_trajectory.trajectory_start = global_plan.start_state_;
                display_trajectory.trajectory.clear();
                display_trajectory.trajectory.push_back(global_plan.trajectory_);
                display_pub.publish(display_trajectory);
            } else {
                ROS_ERROR("Planning failed. Retrying ...");

                // Adjust planning time
                planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
                group.setPlanningTime(planning_time);
                consecutive_failures++;

                if (consecutive_failures >= max_failures) {
                    ROS_WARN("Maximum consecutive planning failures reached. Resetting planning time.");
                    planning_time = 5.0; // Reset to default
                    group.setPlanningTime(planning_time);
                    consecutive_failures = 0;
                }

                // Clear pose targets and continue
                group.clearPoseTargets();
                ros::Duration(0.5).sleep();
                toggle = !toggle;
                continue;
            }
        }

        // Execute the plan
        moveit::core::MoveItErrorCode exec_result = group.execute(global_plan);

        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Execution succeeded. Robot reached the target.");

            // Adjust planning time
            planning_time = std::max(min_planning_time, planning_time - planning_time_decrement);
            group.setPlanningTime(planning_time);
            consecutive_failures = 0;
        } else {
            ROS_ERROR("Execution failed. Retrying...");

            // Adjust planning time
            planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
            group.setPlanningTime(planning_time);
        }

        // Clear pose targets for the next iteration
        group.clearPoseTargets();

        // Pause to allow synchronization
        ros::Duration(0.1).sleep();
        toggle = !toggle;
    }

    // Clear any remaining constraints after exiting the loop
    group.clearPathConstraints();
    ros::shutdown();
    return 0;
}
