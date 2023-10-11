#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_msgs/action/move_robot.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 09.10.2023
 * @about: Action server node
 **/


using namespace std::placeholders;
using MyActionMoveRobot = my_msgs::action::MoveRobot;
using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MyActionMoveRobot>;

class MyRosClass : public rclcpp::Node
{   

    public:
        
        MyRosClass() : Node("action_server_node")
        {   
            std::string action_topic = "/my_action_topic";
            
            action_server = rclcpp_action::create_server<MyActionMoveRobot>(
                this, 
                action_topic,
                std::bind(&MyRosClass::handle_goal, this, _1, _2),
                std::bind(&MyRosClass::handle_cancel, this, _1),
                std::bind(&MyRosClass::handle_accepted, this, _1));

            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Action server node is ready.");
        }

    private:

        std::string service_topic;
        rclcpp_action::Server<MyActionMoveRobot>::SharedPtr action_server;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MyActionMoveRobot::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request.");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
        {
            std::thread{std::bind(&MyRosClass::execute_action, this, _1), goal_handle}.detach();
        }

        void execute_action(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Action started. Executing goal...");

            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<MyActionMoveRobot::Feedback>();
            auto result = std::make_shared<MyActionMoveRobot::Result>();

            rclcpp::Time start_time = this->now();
            rclcpp::Time current_time = this->now();
            rclcpp::Duration elapsed_time = current_time - start_time;
            double seconds = 0.0;

            int progress = 0;
            int loop_hz = 1;
            rclcpp::Rate loop_rate(loop_hz);

            while (rclcpp::ok())
            {      
                if (goal_handle->is_canceling() == true) 
                {
                    result->finish_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                    return;  //break
                }

                feedback->progress = progress;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Action progress: %d percent.", progress);

                current_time = this->now();
                elapsed_time = current_time - start_time;
                seconds = elapsed_time.seconds();
                RCLCPP_INFO(this->get_logger(), "Elapsed time: %f seconds.", seconds);

                if (progress == 100) {
                    RCLCPP_INFO(this->get_logger(), "Progress done.");
                    break;
                }

                progress = progress + 10;
                loop_rate.sleep();
            }

            if (rclcpp::ok() && progress == 100) {
                result->finish_success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Action goal successfully done.");
            }
        }
    

}; // class


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_server_node = std::make_shared<MyRosClass>();
    rclcpp::spin(action_server_node);

    rclcpp::shutdown();
    return 0;
}