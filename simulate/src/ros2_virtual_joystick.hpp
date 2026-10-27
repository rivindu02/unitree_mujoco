#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <atomic>

class Ros2VirtualJoystick : public unitree::common::UnitreeJoystick, public rclcpp::Node {
public:
    Ros2VirtualJoystick() : Node("ros2_virtual_joystick") {
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                lx_.store(static_cast<float>(-msg->linear.y));
                ly_.store(static_cast<float>(msg->linear.x));
                rx_.store(static_cast<float>(-msg->angular.z));
            });
        RCLCPP_INFO(this->get_logger(), "Ros2VirtualJoystick active, listening on /cmd_vel");
    }

    // called at 1000Hz from RobotBridge::run() instead of polling real hardware
    void update() override {
        lx(lx_.load());
        ly(ly_.load());
        rx(rx_.load());
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    std::atomic<float> lx_{0.0f};
    std::atomic<float> ly_{0.0f};
    std::atomic<float> rx_{0.0f};
};
