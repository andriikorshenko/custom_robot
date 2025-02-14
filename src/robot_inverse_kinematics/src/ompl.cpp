#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <mutex>
#include <cmath>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <algorithm>

// Constants
const double TAU = 2 * M_PI;
const double POSE_PRECISION = 1e-3; // Tolerance for comparing poses

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

// Function to validate and correct trajectory timing
void validateAndCorrectTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    double last_time = 0.0;
    for (auto &point : plan.trajectory_.joint_trajectory.points) {
        if (point.time_from_start.toSec() <= last_time) {
            point.time_from_start = ros::Duration(last_time + 0.1); // Increment by 0.1s
            ROS_WARN("Adjusted time_from_start to %.3f seconds to ensure strict increase.", last_time + 0.1);
        }
        last_time = point.time_from_start.toSec();
    }
}

// Function to verify collision objects are in the planning scene
bool verifyCollisionObjects(moveit::planning_interface::PlanningSceneInterface &psi,
                            const std::vector<moveit_msgs::CollisionObject> &objects,
                            double timeout = 5.0)
{
    ros::Time start_time = ros::Time::now();
    while ((ros::Time::now() - start_time).toSec() < timeout) {
        std::vector<std::string> known_objects = psi.getKnownObjectNames();
        bool all_found = true;
        for (const auto &obj : objects) {
            if (std::find(known_objects.begin(), known_objects.end(), obj.id) == known_objects.end()) {
                all_found = false;
                break;
            }
        }
        if (all_found) {
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    return false;
}

// Create your collision objects (e.g., floor, walls, test box)
std::vector<moveit_msgs::CollisionObject> createCollisionObjects()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.reserve(5); // Adjusted from 6 to 5 as one box is commented out

    // 1) Box under the robot
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z = -0.1;
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

    return collision_objects;
}

// Synchronous planning function
bool performSynchronousPlanning(moveit::planning_interface::MoveGroupInterface &group,
                                moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                int max_retries = 10)
{
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        moveit::core::MoveItErrorCode result = group.plan(plan);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_no_wait_code_fixed_with_constraints_and_cache");
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
    group.setMaxVelocityScalingFactor(1.0); // Adjust as needed
    group.setMaxAccelerationScalingFactor(1.0); // Adjust as needed

    group.setPlannerId("PersistentPRM");
    group.setNumPlanningAttempts(5);
    double planning_time = 10;
    group.setPlanningTime(planning_time);

    // If your robot's root link is different, replace "base_link"
    group.setPoseReferenceFrame("base_link");
    ROS_INFO_STREAM("Planner ID: " << group.getPlannerId());
    ROS_INFO_STREAM("Pose reference frame: " << group.getPoseReferenceFrame());

    // *** Removed DisplayTrajectory Publisher ***
    // Removed the following lines to eliminate DisplayTrajectory usage:
    // ros::Publisher display_pub =
    //     nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
    // moveit_msgs::DisplayTrajectory display_trajectory;

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

    // Verify that all collision objects have been added
    if (!verifyCollisionObjects(planning_scene_interface, collision_objects, 5.0)) {
        ROS_WARN("Not all collision objects were added to the planning scene within the timeout.");
    } else {
        ROS_INFO("All collision objects successfully added to the planning scene.");
    }

    // Introduce a warm-up delay for controllers
    ROS_INFO("Waiting for controllers to warm up...");
    ros::Duration(2.0).sleep(); // Adjust based on your robot's requirements

    // 3) Define target poses with the same orientation
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-TAU / 2, 0, 0); // Example orientation (straight)
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = -0.170;
    target_pose1.position.y = 0.069;
    target_pose1.position.z = 0.074;

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = 0.244;
    target_pose2.position.y = -0.226;
    target_pose2.position.z = 0.070;

    // *** Reintroduce Orientation Path Constraints ***
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose1.orientation;
    ocm.absolute_x_axis_tolerance = 0.8;
    ocm.absolute_y_axis_tolerance = 0.8;
    //ocm.absolute_x_axis_tolerance = M_PI * 2;
    //ocm.absolute_z_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = M_PI * 2;
    ocm.weight = 0.3;

    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(path_constraints);
    ROS_INFO("Orientation path constraints set to maintain EEF straight.");

    // *** Initialize Trajectory Cache ***
    // Define a mutex to protect the cache
    std::mutex cache_mutex;

    // Define the cache: PosePair -> Plan
    std::unordered_map<PosePair, moveit::planning_interface::MoveGroupInterface::Plan, PosePairHash> plan_cache;

    // Variables for dynamic planning time
    int consecutive_failures = 0;
    const int max_failures = 5;
    const double planning_time_increment = 1.0;
    const double planning_time_decrement = 0.5;
    const double min_planning_time = 2.0;
    const double max_planning_time = 10.0;

    // *** Pre-Plan and Cache the Initial Trajectory ***
    ROS_INFO("Performing initial planning to populate the cache...");
    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    bool initial_plan_success = performSynchronousPlanning(group, initial_plan, 10);

    if (initial_plan_success) {
        // Create PosePair
        geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
        PosePair initial_pair{current_pose, target_pose1};

        // Validate and correct trajectory timing before caching
        validateAndCorrectTrajectory(initial_plan);

        // Add to cache
        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            plan_cache[initial_pair] = initial_plan;
        }

        ROS_INFO("Initial plan cached with corrected timing.");

        // Execute the initial plan
        moveit::core::MoveItErrorCode exec_result = group.execute(initial_plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Initial execution succeeded.");
        } else {
            ROS_ERROR("Initial execution failed.");
        }
    } else {
        ROS_WARN("Initial planning failed. Proceeding without caching.");
    }

    // Small delay before entering the main loop
    ros::Duration(1.0).sleep();

    // 4) Main planning & execution loop
    bool toggle = true;
    int loop_count = 0;
    ROS_INFO("Starting planning and execution loop with orientation constraints and caching...");
    while (ros::ok()) {
        loop_count++;

        if (loop_count == 1) {
            ROS_INFO("Executing first movement command.");
        }

        // Current robot pose
        geometry_msgs::PoseStamped current_pose_stamped = group.getCurrentPose();
        geometry_msgs::Pose current_pose = current_pose_stamped.pose;

        // Toggle target pose
        geometry_msgs::Pose target_pose = (toggle ? target_pose1 : target_pose2);

        // Create a PosePair
        PosePair current_pair{current_pose, target_pose};

        moveit::planning_interface::MoveGroupInterface::Plan global_plan;
        bool plan_found_in_cache = false;

        {
            // Lock the cache for thread safety
            std::lock_guard<std::mutex> lock(cache_mutex);
            auto it = plan_cache.find(current_pair);
            if (it != plan_cache.end()) {
                ROS_INFO("Found plan in cache. Using cached plan.");
                global_plan = it->second;
                plan_found_in_cache = true;
            }
        }

        if (true) {
            // Set the new goal pose
            group.setStartStateToCurrentState(); // Ensure start state is current
            group.setPoseTarget(target_pose);

            // Time the planning
            ros::Time start_time = ros::Time::now();

            bool planning_success = performSynchronousPlanning(group, global_plan, 10);

            if (!planning_success) {
                // Increase planning time
                planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
                //group.setPlanningTime(planning_time);
                consecutive_failures++;

                if (consecutive_failures >= max_failures) {
                    ROS_WARN("Max consecutive failures. Reset planning time to 5.0s");
                    planning_time = 5.0;
                    //group.setPlanningTime(planning_time);
                    consecutive_failures = 0;
                }

                group.clearPoseTargets();
                ros::Duration(0.5).sleep();
                //toggle = !toggle;
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
                //group.setPlanningTime(planning_time);
                consecutive_failures++;

                if (consecutive_failures >= max_failures) {
                    ROS_WARN("Max consecutive failures. Reset planning time to 5.0s");
                    planning_time = 5.0;
                    //group.setPlanningTime(planning_time);
                    consecutive_failures = 0;
                }

                group.clearPoseTargets();
                ros::Duration(0.5).sleep();
                //toggle = !toggle;
                continue;
            }

            // Validate and correct trajectory timing before caching
            validateAndCorrectTrajectory(global_plan);

            // *** Add the new plan to the cache ***
            {
                std::lock_guard<std::mutex> lock(cache_mutex);
                plan_cache[current_pair] = global_plan;
                ROS_INFO("Plan cached for the current pose pair with corrected timing.");
            }

            ROS_INFO("Plan succeeded and cached. Executing now.");
        } else {
            ROS_INFO("Reusing cached plan. Executing now.");
        }

        // Execute the plan
        moveit::core::MoveItErrorCode exec_result = group.execute(global_plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Execution succeeded. Target reached.");
            toggle = !toggle;
            // Decrease planning time
            planning_time = std::max(min_planning_time, planning_time - planning_time_decrement);
            //group.setPlanningTime(planning_time);
            consecutive_failures = 0;
        } else {
            ROS_ERROR("Execution failed. Retrying...");
            // Increase planning time
            planning_time = std::min(max_planning_time, planning_time + planning_time_increment);
            //group.setPlanningTime(planning_time);
        }

        group.clearPoseTargets();
        ros::Duration(0.5).sleep();
        //toggle = !toggle;
    }

    // Cleanup
    group.clearPathConstraints();
    ros::shutdown();
    return 0;
}
