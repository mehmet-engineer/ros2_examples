#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 06.10.2023
 * @about: Service client node
 **/

class MyRosClass : public rclcpp::Node
{

    private:

        rclcpp::Node::SharedPtr node;
        std::string service_topic;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr srv_client;

    public:

        MyRosClass() : Node("service_client_node")
        {   
            std::string service_topic = "/my_bool_service";
            srv_client = this->create_client<std_srvs::srv::SetBool>(service_topic);
            
            bool is_service_ready = srv_client->wait_for_service(std::chrono::milliseconds(200));

            while (rclcpp::ok()) {
                if (is_service_ready == false) {
                    RCLCPP_WARN(this->get_logger(), "Waiting for %s service ...", service_topic.c_str());
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Connected to service server node.");
                    break;
                }
                is_service_ready = srv_client->wait_for_service(std::chrono::milliseconds(200));
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Service client node is ready.");

            call_service_once();
        }

        void call_service(bool request_data) 
        {
            RCLCPP_INFO(this->get_logger(), "Sending data to server by client...");
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = request_data;

            auto future_result = srv_client->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result_ptr = future_result.get();
                bool success = result_ptr->success;
                std::string message = result_ptr->message;

                if (success == true)
                {
                    RCLCPP_INFO(this->get_logger(), "Service call successfull.");
                    RCLCPP_INFO(this->get_logger(), "Service server message: %s", message.c_str());
                } 
                else {
                    RCLCPP_ERROR(this->get_logger(), "Service success false, failed!");
                }
            } 
            else {
                RCLCPP_ERROR(this->get_logger(), "Service call timed out !! Failed to call %s service !!", service_topic.c_str());
            }
        }

        void call_service_once() 
        {   
            bool data = true;
            int seconds = 3;
            RCLCPP_INFO(this->get_logger(), "Service client will call server with TRUE data after %d seconds...", seconds);
            std::this_thread::sleep_for(std::chrono::seconds(seconds));
            
            call_service(data);
            RCLCPP_INFO(this->get_logger(), "Client process finished.");
        }
        
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto service_client_node = std::make_shared<MyRosClass>();
    //rclcpp::spin(service_client_node);
    
    rclcpp::shutdown();
    return 0;
}