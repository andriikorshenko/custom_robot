#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/conversions.h>
const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    // Node definition and spinner
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Instantiate the MoveGroupInterface and PlanningSceneInterface
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Setup the target position
    group.setPoseReferenceFrame("base_link");

    // Add a collision object for the table
    moveit_msgs::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions = {1.0, 1.0, 0.2}; // Length, width, height of the table

    geometry_msgs::Pose table_pose;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.2; // Table's top is 20 cm below the robot's base
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    planning_scene_interface.applyCollisionObject(table);

    // Move the robot to the "up" position
    ROS_INFO("Moving to 'up' position...");
    group.setNamedTarget("up");
    group.move();

    // Define two target poses
    geometry_msgs::Pose target_pose1, target_pose2;
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 2, 0, 0); // Specific rotation
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.4;
    target_pose1.position.z = 0.4; // Increased height above the table

    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.369;
    target_pose2.position.y = -0.234;
    target_pose2.position.z = 0.086; // Increased height above the table

    // Set planning parameters
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(5);

    // Ping-pong loop between the two target poses
    bool toggle = true;
    while (ros::ok())
    {
        // Update current state before planning
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        if (!current_state->satisfiesBounds())
        {
            ROS_ERROR("Robot state is out of bounds! Skipping this loop iteration.");
            continue;
        }

        group.setStartState(*current_state);

        ROS_INFO("Planning to the next target position...");
        if (toggle)
        {
            group.setPoseTarget(target_pose1, "tool0");
        }
        else
        {
            group.setPoseTarget(target_pose2, "tool0");
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan_to_target;
        moveit::planning_interface::MoveItErrorCode success = group.plan(plan_to_target);

        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            group.execute(plan_to_target);
            ROS_INFO("Reached target position.");
        }
        else
        {
            ROS_ERROR("Failed to plan to target position.");
        }

        // Stop the robot to synchronize its state
        group.stop();

        toggle = !toggle; // Switch to the other target
        sleep(2.0); // Pause for visibility
    }

    ros::shutdown();
    return 0;
}
