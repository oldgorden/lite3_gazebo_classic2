//
// Created by tlab-uav on 24-9-13.
//

#ifndef JOYSTICKINPUT_H
#define JOYSTICKINPUT_H

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32.hpp>


class JoystickInput final : public rclcpp::Node {
public:
    JoystickInput();

    ~JoystickInput() override = default;

private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_mode_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};


#endif //JOYSTICKINPUT_H
