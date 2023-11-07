#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 03.10.2023
 * @about: Timer tutorial node
 **/

class MyROSClass : public rclcpp::Node
{

  private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timer_publisher;
    std::chrono::seconds timer_period;
    rclcpp::TimerBase::SharedPtr timer;


  public:

    MyROSClass() : Node("timer_tutorial_node")
    {
      // Publisher
      timer_publisher = this->create_publisher<std_msgs::msg::String>("/cpp_nodes/timer_publisher_topic", 10);

      // Timer
      auto timer_period = std::chrono::milliseconds(2000);
      timer = this->create_wall_timer(timer_period, std::bind(&MyROSClass::timer_callback, this));

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    void timer_callback()
    {
      auto timer_msg = std_msgs::msg::String();
      timer_msg.data = "Hello ROS 2 developer!";
      timer_publisher->publish(timer_msg);
      RCLCPP_INFO(this->get_logger(), "Timer is publishing...");
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("timer_tutorial_node"), "Node initialized.");
  auto node = std::make_shared<MyROSClass>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}