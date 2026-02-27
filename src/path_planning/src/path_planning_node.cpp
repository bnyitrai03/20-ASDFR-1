// /////////////////////////////////////////////////////////////////////////////////
// Authors:   Bence Nyitrai
// Group:     2x
// License:   Apache License
//
// Brief:     
// /////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "example_interfaces/msg/float64.hpp"

enum class MotionState {
    IDLE,
    FORWARD,
    TURN_RIGHT,
    TURN_LEFT,
    STOP
};

class PathPlanningNode : public rclcpp::Node
{
public:
    PathPlanningNode()
        : Node("path_planning_node"), state_start_time_(this->now())
    {   
        // Publish the motor speeds
        pub_left_  = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/left_motor/setpoint_vel",  10);
        pub_right_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/right_motor/setpoint_vel", 10);

        // Subscribe to the coordinates
        sub_coord_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/object_coordinate", 10,
            std::bind(&PathPlanningNode::detection_callback, this,
                      std::placeholders::_1));
        
        // Create timer which drives the state machine
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&PathPlanningNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),"path_planning_node started");
    }

private:
    void detection_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (msg->z == 1.0 && state_ == MotionState::IDLE){
            state_ = MotionState::FORWARD;
            state_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Object detected, starting sequence");
        }
    }

    void timer_callback()
    {
        example_interfaces::msg::Float64 left, right;

        // Set motor values based on current state
        switch (state_) {
            case MotionState::FORWARD:
                left.data = -3;
                right.data = 3;
                break;

            case MotionState::TURN_RIGHT:
                left.data = -3;
                right.data = -3;
                break;

            case MotionState::TURN_LEFT:
                left.data = 3;
                right.data = 3;
                break;

            case MotionState::STOP:
                left.data = 0.0;
                right.data = 0.0;
                break;

            default:
                return;
        }

        // Publish every ms
        pub_left_->publish(left);
        pub_right_->publish(right);

        state_transition();
    }

    void state_transition(){
        // Check if 5000 ms elapsed
        if ((this->now() - state_start_time_) >= rclcpp::Duration(DURATION)) {
            // Move to next state
            switch (state_) {
                case MotionState::FORWARD:
                    state_ = MotionState::TURN_RIGHT;
                    break;
                case MotionState::TURN_RIGHT:
                    state_ = MotionState::TURN_LEFT;
                    break;
                case MotionState::TURN_LEFT:
                    state_ = MotionState::STOP;
                    break;
                case MotionState::STOP:
                    state_ = MotionState::IDLE;
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "Unknown state");
                    break;
            }

            state_start_time_ = this->now();
        }
    }

    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_left_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_right_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_coord_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time state_start_time_;
    const std::chrono::milliseconds DURATION{5000};
    MotionState state_ = MotionState::IDLE;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanningNode>());
    rclcpp::shutdown();
    return 0;
}