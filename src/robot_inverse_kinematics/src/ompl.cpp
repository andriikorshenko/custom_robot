#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_planner_straight_eef");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Initialize MoveIt
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setMaxVelocityScalingFactor(0.5); // Smooth motion
    group.setMaxAccelerationScalingFactor(0.5);

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    group.setPoseReferenceFrame("base_link");

    // Add a box under the robot to the planning scene
    moveit_msgs::CollisionObject box;
    box.header.frame_id = "base_link";
    box.id = "box";

    // Define the box dimensions
    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::SolidPrimitive::BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.5; // Length (X)
    box_primitive.dimensions[1] = 0.5; // Width (Y)
    box_primitive.dimensions[2] = 0.1; // Height (Z)

    // Define the box pose
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05; // Slightly below the robot's base_link

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    // Add the box to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(box);
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Added a box under the robot.");

    // Define target poses
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 2, 0, 0); // Specific rotation
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.4;
    target_pose1.position.z = 0.4;

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.369;
    target_pose2.position.y = -0.234;
    target_pose2.position.z = 0.140;

    // Add an orientation constraint to keep the EEF straight
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose1.orientation; // Desired orientation
    ocm.absolute_x_axis_tolerance = 0.05;      // Tighter tolerances
    ocm.absolute_y_axis_tolerance = 0.05;
    ocm.absolute_z_axis_tolerance = 0.05;
    ocm.weight = 1.0;

    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(path_constraints);

    // Planning and execution loop
    bool toggle = true;
    while (ros::ok())
    {
        // Set the start state
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        group.setStartState(*current_state);

        // Toggle between target poses
        geometry_msgs::Pose target_pose = toggle ? target_pose1 : target_pose2;
        group.setPoseTarget(target_pose);

        // Plan the trajectory using OMPL
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
        ros::Duration(2.0).sleep(); // Pause for visibility
    }

    // Clean up path constraints when done
    group.clearPathConstraints();

    ros::shutdown();
    return 0;
}
