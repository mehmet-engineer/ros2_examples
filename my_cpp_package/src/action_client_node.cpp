#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_msgs/action/move_robot.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 09.10.2023
 * @about: Action client node
 **/


using namespace std::placeholders;
using MyActionMoveRobot = my_msgs::action::MoveRobot;
using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MyActionMoveRobot>;

class MyRosClass : public rclcpp::Node
{   

    public:
        
        MyRosClass() : Node("action_client_node")
        {   
            std::string action_topic = "/my_action_topic";
            
            action_client = rclcpp_action::create_client<MyActionMoveRobot>(this, action_topic);

            bool is_action_server_ready = action_client->wait_for_action_server(std::chrono::milliseconds(200));

            while (rclcpp::ok()) {
                if (is_action_server_ready == false) {
                    RCLCPP_WARN(this->get_logger(), "Waiting for %s action server ...", service_topic.c_str());
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Connected to action server node.");
                    break;
                }
                is_action_server_ready = action_client->wait_for_action_server(std::chrono::milliseconds(200));
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Action client node is ready.");

            execute_once();
        }

    private:

        std::string service_topic;
        rclcpp_action::Client<MyActionMoveRobot>::SharedPtr action_client;

        void send_goal(std::vector<double> pos_vector)
        {
            RCLCPP_INFO(this->get_logger(), "Sending action goal...");
            
            auto goal_msg = MyActionMoveRobot::Goal();
            goal_msg.target_position = pos_vector;

            auto send_goal_options = rclcpp_action::Client<MyActionMoveRobot>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&MyRosClass::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&MyRosClass::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&MyRosClass::result_callback, this, _1);
            action_client->async_send_goal(goal_msg, send_goal_options);
        }

        void goal_response_callback(const GoalHandleMoveRobot::SharedPtr & goal_handle)
        {
            if (!goal_handle) 
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server!");
            } 
            else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by action server, waiting for result.");
            }
        }

        void feedback_callback(
            GoalHandleMoveRobot::SharedPtr, 
            const std::shared_ptr<const MyActionMoveRobot::Feedback> feedback)
        {   
            int progress = feedback->progress;
            RCLCPP_INFO(this->get_logger(), "Feedback received, Progress: %d", progress);
        }

        void result_callback(const GoalHandleMoveRobot::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Action Goal successfully done.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Action Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Action Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown error occurred in the result of action!");
                    return;
            }
        }

        void execute_once()
        {   
            int seconds = 3;
            RCLCPP_INFO(this->get_logger(), "Action goal will be sent in %d seconds...", seconds);
            std::this_thread::sleep_for(std::chrono::seconds(seconds));

            auto target_pos_vector = std::vector<double>(6);
            for (size_t i=0; i<6; i++) {
                target_pos_vector[i] = 0.0;
            }

            send_goal(target_pos_vector);
        }
    

}; // class


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_client_node = std::make_shared<MyRosClass>();
    //rclcpp::spin(action_client_node);

    rclcpp::shutdown();
    return 0;
}