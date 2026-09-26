#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

bool move_robot(std::vector<double> pos, MoveGroupInterface &move_group_iface, rclcpp::Logger logger) {
  
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

int main(int argc, char * argv[])
{
  // start rclcpp ros client 
  rclcpp::init(argc, argv);
  
  // get launch parameters with NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // create node with smart pointer
  auto const node = std::make_shared<rclcpp::Node>("robot_sim_node", node_options);
  auto const logger = rclcpp::get_logger("robot_sim");
  RCLCPP_INFO(logger, "C++ MoveIt Node initializing...");

  // wait for parameter reading...
  rclcpp::sleep_for(std::chrono::seconds(1));

  // start move group interface
  std::string plan_group = "ur5_group";
  auto move_group_iface = MoveGroupInterface(node, plan_group);

  // set speed and acc
  move_group_iface.setMaxVelocityScalingFactor(0.1);
  move_group_iface.setMaxAccelerationScalingFactor(0.1);

  // set example target joint positions
  std::vector<double> home_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> down_pos = {0.0, 0.0, 1.57, 0.0, -1.57, 0.0};

  // call move_robot function
  bool result1 = move_robot(home_pos, move_group_iface, logger);
  bool result2 = move_robot(down_pos, move_group_iface, logger);

  // write log
  RCLCPP_INFO(logger, "Node done.");

  rclcpp::shutdown();
  return 0;
}