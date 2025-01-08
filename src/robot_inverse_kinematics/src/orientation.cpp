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

  move_group.setPoseReferenceFrame("base_link");
  move_group.setPlanningTime(10.0);

  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose first_goal_pose;
  first_goal_pose.position.x = 0.0;
  first_goal_pose.position.y = 0.4;
  first_goal_pose.position.z = 0.3;
  first_goal_pose.orientation.x = 0.0;
  first_goal_pose.orientation.y = 0.0;
  first_goal_pose.orientation.z = 0.0;
  first_goal_pose.orientation.w = 1.0;

  move_group.setPoseTarget(first_goal_pose, "tool0");

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "tool0";
  ocm.header.frame_id = "base_link";
  ocm.orientation = first_goal_pose.orientation;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(path_constraints);

  moveit::planning_interface::MoveGroupInterface::Plan first_plan;
  bool first_plan_success = (move_group.plan(first_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (first_plan_success)
  {
    ROS_INFO("Planning to the first point successful. Executing...");
    moveit::planning_interface::MoveItErrorCode exec_result = move_group.execute(first_plan);

    if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("Execution failed for the first point.");
    }
  }
  else
  {
    ROS_WARN("Planning to the first point failed.");
    ros::shutdown();
    return 0;
  }

  move_group.clearPathConstraints();
  ros::shutdown();
  return 0;
}