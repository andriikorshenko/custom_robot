#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread> // To get hardware concurrency

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_planner_with_optimizations");
    ros::NodeHandle nh;

    if (!ros::master::check()) {
        ROS_ERROR("ROS Master is not running. Exiting...");
        return -1;
    }

    // Dynamically determine the maximum number of threads
    unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency() - 1); // Leave 1 core for OS tasks
    ROS_INFO("Using %u threads for AsyncSpinner.", num_threads);

    ros::AsyncSpinner spinner(num_threads); // Enable multi-threading
    spinner.start();
    ros::Duration(1.0).sleep();

    // Initialize MoveIt
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Maximize velocity and acceleration
    group.setMaxVelocityScalingFactor(1.0); // Maximum velocity
    group.setMaxAccelerationScalingFactor(1.0); // Maximum acceleration

    // Use RRTConnect planner
    group.setPlannerId("RRTConnectkConfigDefault");

    // Dynamically set planner parameters
    nh.setParam("/ompl_planner_configs/RRTConnectkConfigDefault/range", 0.5); // Increase step size
    nh.setParam("/ompl_planner_configs/RRTConnectkConfigDefault/goal_bias", 0.1); // Favor the goal
    nh.setParam("/ompl_planner_configs/RRTConnectkConfigDefault/threads", static_cast<int>(num_threads)); // Cast to int

    // Set planning time
    group.setPlanningTime(5.0);

    // Initialize publisher for trajectory visualization
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    group.setPoseReferenceFrame("base_link");

    // Monitor the planning scene
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
    if (!psm.requestPlanningSceneState()) {
        ROS_WARN("Failed to request the planning scene state. Proceeding anyway.");
    }

    // Define collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Add the box under the robot
    moveit_msgs::CollisionObject box;
    box.header.frame_id = "base_link";
    box.id = "box";

    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::SolidPrimitive::BOX;
    box_primitive.dimensions = {0.5, 0.5, 0.1};

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05;

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;
    collision_objects.push_back(box);

    // Add other collision objects (walls)
    // Front wall
    moveit_msgs::CollisionObject front_wall;
    front_wall.header.frame_id = "base_link";
    front_wall.id = "front_wall";

    shape_msgs::SolidPrimitive front_wall_primitive;
    front_wall_primitive.type = shape_msgs::SolidPrimitive::BOX;
    front_wall_primitive.dimensions = {0.5, 0.02, 0.5};

    geometry_msgs::Pose front_wall_pose;
    front_wall_pose.orientation.w = 1.0;
    front_wall_pose.position.x = 0.0;
    front_wall_pose.position.y = 0.3; // Positioned in front of the cube
    front_wall_pose.position.z = 0.25;

    front_wall.primitives.push_back(front_wall_primitive);
    front_wall.primitive_poses.push_back(front_wall_pose);
    front_wall.operation = front_wall.ADD;
    collision_objects.push_back(front_wall);

    // Back wall
    moveit_msgs::CollisionObject back_wall;
    back_wall.header.frame_id = "base_link";
    back_wall.id = "back_wall";

    shape_msgs::SolidPrimitive back_wall_primitive;
    back_wall_primitive.type = shape_msgs::SolidPrimitive::BOX;
    back_wall_primitive.dimensions = {0.5, 0.02, 0.5};

    geometry_msgs::Pose back_wall_pose;
    back_wall_pose.orientation.w = 1.0;
    back_wall_pose.position.x = 0.0;
    back_wall_pose.position.y = -0.3; // Positioned behind the cube
    back_wall_pose.position.z = 0.25;

    back_wall.primitives.push_back(back_wall_primitive);
    back_wall.primitive_poses.push_back(back_wall_pose);
    back_wall.operation = back_wall.ADD;
    collision_objects.push_back(back_wall);

    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("Added collision objects. Waiting for the planning scene to update...");
    ros::Duration(1.0).sleep();

    // Verify collision objects
    auto known_objects = planning_scene_interface.getKnownObjectNames();
    if (known_objects.empty()) {
        ROS_WARN("No collision objects found in the planning scene. Check the setup.");
    } else {
        ROS_INFO("Collision objects successfully added: %lu objects.", known_objects.size());
    }

    // Define target poses
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 2, 0, 0); // Keep EEF orientation straight
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.272;
    target_pose1.position.y = 0.004;
    target_pose1.position.z = 0.110;

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.234;
    target_pose2.position.y = 0.155;
    target_pose2.position.z = 0.033;

    // Loosen orientation constraints
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose1.orientation;
    ocm.absolute_x_axis_tolerance = 0.2; // Loosen tolerances
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;

    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(path_constraints);

    // Planning and execution loop
    bool toggle = true;
    ROS_INFO("Starting planning and execution loop...");
    while (ros::ok())
    {
        group.setStartStateToCurrentState();

        geometry_msgs::Pose target_pose = toggle ? target_pose1 : target_pose2;
        group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan global_plan;
        auto success = group.plan(global_plan);

        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Plan succeeded. Executing trajectory...");
            auto execution_result = group.execute(global_plan); // Synchronous execution

            if (execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Execution succeeded. Robot reached the target.");
            }
            else
            {
                ROS_ERROR("Execution failed. Retrying...");
            }
        }
        else
        {
            ROS_ERROR("Planning failed. Retrying...");
        }

        group.stop();
        ros::Duration(0.5).sleep(); // Pause to allow synchronization
        toggle = !toggle;
    }

    group.clearPathConstraints();
    ros::shutdown();
    return 0;
}
