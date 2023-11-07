#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 03.10.2023
 * @about: Subscriber node
 **/

class MyROSClass : public rclcpp::Node
{

  private:
    
    double position_value = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscription;
    sensor_msgs::msg::JointState js_message;


  public:

    MyROSClass() : Node("subscriber_node")
    {

      // Subscriber
      js_subscription = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
                        std::bind(&MyROSClass::subscriber_callback, this, std::placeholders::_1));
      auto js_message = sensor_msgs::msg::JointState();

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    void subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    { 
      position_value = msg->position[0];
      RCLCPP_INFO(this->get_logger(), "JS subscriber position received: '%f'", position_value);
    }


};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Node initialized.");
  auto node = std::make_shared<MyROSClass>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}