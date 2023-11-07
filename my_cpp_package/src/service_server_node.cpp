#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 06.10.2023
 * @about: Service server node
 **/

class MyServiceNode : public rclcpp::Node
{
    public:

        MyServiceNode() : Node("service_server_node")
        { 
            std::string service_topic = "/my_bool_service";
            service = create_service<std_srvs::srv::SetBool>(service_topic, std::bind(&MyServiceNode::service_callback, this, 
                                                             std::placeholders::_1, std::placeholders::_2));
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(get_logger(), "Service server node is ready.");
        }

    private:

        std::string service_topic;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;
    
        void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            bool req_data = request->data;
            std::string message = "";

            if (req_data == true) {
                message = "Service called with TRUE";
                RCLCPP_INFO(get_logger(), message.c_str());
            }
            else {
                message = "Service called with FALSE";
                RCLCPP_INFO(get_logger(), message.c_str());
            }
            
            response->message = message;
            response->success = true;
        }
        
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto service_server_node = std::make_shared<MyServiceNode>();
    rclcpp::spin(service_server_node);

    rclcpp::shutdown();
    return 0;
}