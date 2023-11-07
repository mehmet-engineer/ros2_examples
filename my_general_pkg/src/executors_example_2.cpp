#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 07.11.2023
 * @about: executors and callback groups example, publisher and subscriber are working synchronously but subscriber callbacks are not.
 **/

class PublisherClass : public rclcpp::Node
{
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timer_publisher;
    std::chrono::seconds timer_period;
    rclcpp::TimerBase::SharedPtr timer;

  public:

    PublisherClass() : Node("publisher_executor_node")
    {
      timer_publisher = this->create_publisher<std_msgs::msg::String>("/publisher_executor", 10);
      auto timer_period = std::chrono::milliseconds(1000);
      timer = this->create_wall_timer(timer_period, std::bind(&PublisherClass::timer_callback, this));
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void timer_callback()
    {
      auto timer_msg = std_msgs::msg::String();
      timer_msg.data = "Hello, this message comes from publisher_executor_node";
      timer_publisher->publish(timer_msg);
      RCLCPP_INFO(this->get_logger(), "publisher_executor_node is publishing...");
    }
};

class SubscriberClass : public rclcpp::Node
{
  private:
    std::string str_1_value = "";
    std::string str_2_value = "";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr other_subscription;

  public:

    SubscriberClass() : Node("subscriber_node")
    {
      string_subscription = this->create_subscription<std_msgs::msg::String>("/publisher_executor", 10, 
                        std::bind(&SubscriberClass::subscriber_callback_1, this, std::placeholders::_1));
      other_subscription = this->create_subscription<std_msgs::msg::String>("/publisher_executor", 10, 
                        std::bind(&SubscriberClass::subscriber_callback_2, this, std::placeholders::_1));
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void subscriber_callback_1(const std_msgs::msg::String::SharedPtr msg)
    {
      str_1_value = msg->data;
      RCLCPP_INFO(this->get_logger(), "subscriber_callback_1 receiving...");
    }

    void subscriber_callback_2(const std_msgs::msg::String::SharedPtr msg)
    { 
      RCLCPP_INFO(this->get_logger(), "subscriber_callback_2 started to own process...");
      // heavy process !!
      std::this_thread::sleep_for(std::chrono::seconds(5));
      // heavy process !!

      str_2_value = msg->data;
      RCLCPP_INFO(this->get_logger(), "subscriber_callback_2 receiving...");
    }
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto pub_node = std::make_shared<PublisherClass>();
  auto sub_node = std::make_shared<SubscriberClass>();
  
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}