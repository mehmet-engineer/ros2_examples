#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

bool move_robot_joint(std::vector<double> pos, MoveGroupInterface &move_group_iface, rclcpp::Logger logger) {
  
  // set target and error check
  if (!move_group_iface.setJointValueTarget(pos)) {
    RCLCPP_ERROR(logger, "Joint Target Error!");
    return false;
  }

  // get plan result
  MoveGroupInterface::Plan my_plan;
  bool success = (move_group_iface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // if plan is successfull, move robot
  if (success) {
    RCLCPP_INFO(logger, "Robot moving...");
    move_group_iface.execute(my_plan);
    RCLCPP_INFO(logger, "Target reached.");
    return true;
  } 
  else {
    RCLCPP_ERROR(logger, "Target plan failed.");
    return false;
  }

}

bool move_robot_cartesian(float x, float y, float z, MoveGroupInterface &move_group_iface, rclcpp::Logger logger) {
  
  geometry_msgs::msg::Pose current_pose = move_group_iface.getCurrentPose().pose;
  geometry_msgs::msg::Pose target_pose = current_pose;
  
  target_pose.position.x = current_pose.position.x + x;
  target_pose.position.y = current_pose.position.y + y;
  target_pose.position.z = current_pose.position.z + z;
  target_pose.orientation.w = current_pose.orientation.w;
  target_pose.orientation.x = current_pose.orientation.x;
  target_pose.orientation.y = current_pose.orientation.y;
  target_pose.orientation.z = current_pose.orientation.z;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  // interpolation step
  const double eef_step = 0.01;

  // collision check
  const double jump_threshold = 0.0;

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_iface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction < 0.7)
  {
      RCLCPP_INFO(logger, "FRACTION: %.2f", fraction*100);
      return false;
  }
  
  move_group_iface.execute(trajectory);

  return true;
}

int main(int argc, char * argv[])
{
  // start rclcpp ros client 
  rclcpp::init(argc, argv);
  
  // get launch parameters with NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // create node and executor
  auto const node = std::make_shared<rclcpp::Node>("robot_sim_node", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto const logger = rclcpp::get_logger("robot_sim");
  RCLCPP_INFO(logger, "C++ MoveIt Node initializing...");

  // wait for parameter reading...
  rclcpp::sleep_for(std::chrono::seconds(1));

  // start move group interface
  std::string plan_group = "ur5_group";
  auto move_group_iface = MoveGroupInterface(node, plan_group);

  // set speed and acc
  move_group_iface.setMaxVelocityScalingFactor(0.2);
  move_group_iface.setMaxAccelerationScalingFactor(0.2);

  // set example target joint positions
  std::vector<double> home_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> down_pos = {0.0, 0.0, 1.57, 0.0, -1.57, 0.0};

  // call move_robot_joint function
  bool result1 = move_robot_joint(home_pos, move_group_iface, logger);
  bool result2 = move_robot_joint(down_pos, move_group_iface, logger);

  // call move_robot_cartesian function
  float x = 0.0;
  float y = 0.0;
  float z = 0.2;
  bool result3 = move_robot_cartesian(x, y, z, move_group_iface, logger);

  // call move_robot_cartesian function
  z = -0.2;
  bool result4 = move_robot_cartesian(x, y, z, move_group_iface, logger);

  // write log
  RCLCPP_INFO(logger, "Node done.");

  rclcpp::shutdown();
  return 0;
}