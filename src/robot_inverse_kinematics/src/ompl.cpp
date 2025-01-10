#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_planner_with_orientation_constraint");
    ros::NodeHandle nh;

    if (!ros::master::check()) {
        ROS_ERROR("ROS Master is not running. Exiting...");
        return -1;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Initialize MoveIt
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setMaxVelocityScalingFactor(1);
    group.setMaxAccelerationScalingFactor(1);

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    group.setPoseReferenceFrame("base_link");

    // Monitor the planning scene
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
    psm.requestPlanningSceneState();

    // Define collision objects: cube and walls
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Add the box under the robot
    moveit_msgs::CollisionObject box;
    box.header.frame_id = "base_link";
    box.id = "box";

    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::SolidPrimitive::BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.5; // Length (X)
    box_primitive.dimensions[1] = 0.5; // Width (Y)
    box_primitive.dimensions[2] = 0.1; // Height (Z)

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05;

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;
    collision_objects.push_back(box);

    // Add the front wall
    moveit_msgs::CollisionObject front_wall;
    front_wall.header.frame_id = "base_link";
    front_wall.id = "front_wall";

    shape_msgs::SolidPrimitive front_wall_primitive;
    front_wall_primitive.type = shape_msgs::SolidPrimitive::BOX;
    front_wall_primitive.dimensions.resize(3);
    front_wall_primitive.dimensions[0] = 0.5; // Width (X)
    front_wall_primitive.dimensions[1] = 0.02; // Thickness (Y)
    front_wall_primitive.dimensions[2] = 0.5; // Height (Z)

    geometry_msgs::Pose front_wall_pose;
    front_wall_pose.orientation.w = 1.0;
    front_wall_pose.position.x = 0.0;
    front_wall_pose.position.y = 0.3; // Positioned 30 cm in front of the cube
    front_wall_pose.position.z = 0.25;

    front_wall.primitives.push_back(front_wall_primitive);
    front_wall.primitive_poses.push_back(front_wall_pose);
    front_wall.operation = front_wall.ADD;
    collision_objects.push_back(front_wall);

    // Add the back wall
    moveit_msgs::CollisionObject back_wall;
    back_wall.header.frame_id = "base_link";
    back_wall.id = "back_wall";

    shape_msgs::SolidPrimitive back_wall_primitive;
    back_wall_primitive.type = shape_msgs::SolidPrimitive::BOX;
    back_wall_primitive.dimensions.resize(3);
    back_wall_primitive.dimensions[0] = 0.5; // Width (X)
    back_wall_primitive.dimensions[1] = 0.02; // Thickness (Y)
    back_wall_primitive.dimensions[2] = 0.5; // Height (Z)

    geometry_msgs::Pose back_wall_pose;
    back_wall_pose.orientation.w = 1.0;
    back_wall_pose.position.x = 0.0;
    back_wall_pose.position.y = -0.3; // Positioned 30 cm behind the cube
    back_wall_pose.position.z = 0.25;

    back_wall.primitives.push_back(back_wall_primitive);
    back_wall.primitive_poses.push_back(back_wall_pose);
    back_wall.operation = back_wall.ADD;
    collision_objects.push_back(back_wall);

    // Add all collision objects to the planning scene
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Added the cube and two walls. Waiting for updates in the planning scene...");

    // Wait for objects to appear in the planning scene
    bool objects_added = false;
    for (int i = 0; i < 10; ++i) {
        auto updated_objects = planning_scene_interface.getKnownObjectNames();
        if (std::find(updated_objects.begin(), updated_objects.end(), "box") != updated_objects.end() &&
            std::find(updated_objects.begin(), updated_objects.end(), "front_wall") != updated_objects.end() &&
            std::find(updated_objects.begin(), updated_objects.end(), "back_wall") != updated_objects.end()) {
            ROS_INFO("Cube and walls successfully added to the planning scene.");
            objects_added = true;
            break;
        }
        ros::Duration(0.5).sleep(); // Retry every 0.5 seconds
    }

    if (!objects_added) {
        ROS_WARN("Failed to add all obstacles to the planning scene. Proceeding anyway...");
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

    // Add an orientation constraint to keep the EEF straight
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose1.orientation; // Desired orientation
    ocm.absolute_x_axis_tolerance = 0.01;      // Tight tolerances
    ocm.absolute_y_axis_tolerance = 0.01;
    ocm.absolute_z_axis_tolerance = 0.01;
    ocm.weight = 1.0;

    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(path_constraints);

    // Planning and execution loop
    bool toggle = true;
    ROS_INFO("Starting planning and execution loop...");
    while (ros::ok())
    {
        // Set the start state
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        group.setStartState(*current_state);

        // Toggle between target poses
        geometry_msgs::Pose target_pose = toggle ? target_pose1 : target_pose2;
        group.setPoseTarget(target_pose);

        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan global_plan;
        moveit::planning_interface::MoveItErrorCode success = group.plan(global_plan);

        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Plan succeeded. Executing...");
            group.execute(global_plan);
        }
        else
        {
            ROS_ERROR("Planning failed.");
        }

        // Pause for synchronization
        group.stop();
        ros::Duration(1.0).sleep();

        // Toggle target pose
        toggle = !toggle;
        ros::Duration(2.0).sleep();
    }

    // Clean up path constraints
    group.clearPathConstraints();

    ros::shutdown();
    return 0;
}

