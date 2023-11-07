#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 09.10.2023
 * @about: Using parameters node
 **/

class MyROSClass : public rclcpp::Node
{

  private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timer_publisher;
    std::chrono::seconds timer_period;
    rclcpp::TimerBase::SharedPtr timer;


  public:

    MyROSClass() : Node("using_parameters_node")
    {

      // Declaring parameters in this node 
      this->declare_parameter("my_frame", "world");
      this->declare_parameter("my_count", 8);

      // Reset parameters in this node
      int new_value = 12;
      auto new_param = rclcpp::Parameter("my_count", new_value);
      this->set_parameter(new_param);

      // Get or set parameters from another node
      // service_topic = "using_parameters_node/my_frame";
      // auto get_param_client = this->create_client<rcl_interfaces::srv::GetParameters>(service_topic);
      // auto set_param_client = this->create_client<rcl_interfaces::srv::SetParameters>(service_topic);

      // Publisher
      timer_publisher = this->create_publisher<std_msgs::msg::String>("/using_parameters_topic", 10);
      auto timer_period = std::chrono::milliseconds(2000);
      timer = this->create_wall_timer(timer_period, std::bind(&MyROSClass::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "using_parameters_node is ready.");
      std::this_thread::sleep_for(std::chrono::seconds(2));

    }

    void timer_callback()
    {
      std::string my_frame = this->get_parameter("my_frame").as_string();
      int my_count = this->get_parameter("my_count").as_int();

      auto timer_msg = std_msgs::msg::String();
      timer_msg.data = "Received parameter: " + my_frame;
      timer_publisher->publish(timer_msg);
      RCLCPP_INFO(this->get_logger(), "Received my_frame: %s", my_frame.c_str());
      RCLCPP_INFO(this->get_logger(), "Received my_count: %d", my_count);
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyROSClass>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}