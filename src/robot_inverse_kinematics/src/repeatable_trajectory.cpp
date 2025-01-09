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
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setMaxVelocityScalingFactor(1); // Reduced velocity for smoother motion
    group.setMaxAccelerationScalingFactor(1); // Reduced acceleration for smoother motion

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    group.setPoseReferenceFrame("base_link");

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
    target_pose2.position.z = 0.086;

    group.setPlanningTime(15.0);
    group.setNumPlanningAttempts(10);

    bool toggle = true;
    while (ros::ok())
    {
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        if (!current_state->satisfiesBounds())
        {
            ROS_ERROR("Robot state is out of bounds! Skipping this loop iteration.");
            continue;
        }

        group.setStartState(*current_state);

        ROS_INFO("Planning to the next target position...");
        geometry_msgs::Pose start_pose = toggle ? target_pose1 : target_pose2;
        geometry_msgs::Pose end_pose = toggle ? target_pose2 : target_pose1;

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(start_pose);
        waypoints.push_back(end_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.01; // Stricter jump threshold
        const double eef_step = 0.001;     // Smaller step for finer granularity
        double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.9) // Require higher completion fraction for execution
        {
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;

            moveit::planning_interface::MoveItErrorCode execution_result = group.execute(cartesian_plan);
            if (execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Execution succeeded.");
            }
            else
            {
                ROS_WARN("Execution failed.");
            }
        }
        else
        {
            ROS_ERROR("Failed to compute a sufficient Cartesian path. Fraction: %.2f", fraction);
        }

        group.stop();
        ros::Duration(1.0).sleep(); // Pause for synchronization
        toggle = !toggle;
        ros::Duration(2.0).sleep(); // Pause for visibility
    }

    ros::shutdown();
    return 0;
}
