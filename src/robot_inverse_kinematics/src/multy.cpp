#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orientation_constraint_example");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Set velocity and acceleration scaling factors to maximum (1.0)
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // Set basic parameters
  move_group.setPoseReferenceFrame("base_link");
  move_group.setPlanningTime(5.0); // Reduced planning time
  // Optionally set a faster planner
  // move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setStartStateToCurrentState();

  // Define a desired orientation for all goals
  geometry_msgs::Quaternion desired_orientation;
  desired_orientation.x = 1.0;
  desired_orientation.y = 0.0;
  desired_orientation.z = 0.0;
  desired_orientation.w = 0.0;

  // Define the list of goal poses
  std::vector<geometry_msgs::Pose> goal_poses;
  {
    geometry_msgs::Pose pose;
    
    // (1) First goal pose
    pose.position.x = 0.0;
    pose.position.y = 0.4;
    pose.position.z = 0.181;
    pose.orientation = desired_orientation;
    goal_poses.push_back(pose);
    
    // (2) Second goal pose
    pose.position.x = -0.376;
    pose.position.y = -0.243;
    pose.position.z = 0.356;
    pose.orientation = desired_orientation;
    goal_poses.push_back(pose);
  }

  // Create an orientation constraint
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "tool0";
  ocm.header.frame_id = "base_link";
  ocm.orientation = desired_orientation;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = 0.2;
  ocm.weight = 1.0;

  // Create the path constraints object
  moveit_msgs::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(ocm);

  // Optionally set the path constraints
  move_group.setPathConstraints(path_constraints);

  // Main infinite loop
  ros::Duration time_offset(0.0); // Initial time offset

  while (ros::ok())
  {
    for (std::size_t i = 0; i < goal_poses.size(); ++i)
    {
      // Update start state to current for each new plan
      move_group.setStartStateToCurrentState();

      // Set the next pose target
      move_group.setPoseTarget(goal_poses[i], "tool0");

      // Plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!plan_success)
      {
        ROS_WARN_STREAM("Planning failed at point #" << i + 1);
        continue; // Skip to the next waypoint
      }

      // Adjust time_from_start in the trajectory
      if (!plan.trajectory_.joint_trajectory.points.empty())
      {
        ros::Duration point_time_gap(0.1); // Reduced time gap for faster movements
        for (std::size_t j = 0; j < plan.trajectory_.joint_trajectory.points.size(); ++j)
        {
          plan.trajectory_.joint_trajectory.points[j].time_from_start += time_offset;

          // Ensure minimum time gap
          if (j > 0)
          {
            ros::Duration prev_time = plan.trajectory_.joint_trajectory.points[j - 1].time_from_start;
            if (plan.trajectory_.joint_trajectory.points[j].time_from_start <= prev_time)
            {
              plan.trajectory_.joint_trajectory.points[j].time_from_start = prev_time + point_time_gap;
            }
          }
        }
        // Update time offset for next trajectory
        time_offset = plan.trajectory_.joint_trajectory.points.back().time_from_start;
      }

      // Execute the plan
      ROS_INFO_STREAM("Planning to point #" << i + 1 << " successful. Executing...");
      moveit::planning_interface::MoveItErrorCode exec_result = move_group.execute(plan);

      if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_WARN_STREAM("Execution failed at point #" << i + 1);
      }
    }
  }

  move_group.clearPathConstraints();
  ros::shutdown();
  return 0;
}