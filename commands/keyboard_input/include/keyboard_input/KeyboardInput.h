//
// Created by biao on 24-9-11.
//


#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <termios.h>

class KeyboardInput final : public rclcpp::Node {
public:
    KeyboardInput();

    ~KeyboardInput() override {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

private:
    void timer_callback();
    void handle_key(char key);
    void publish_mode(int mode);
    void stop_motion();

    static bool kbhit();

    geometry_msgs::msg::Twist twist_msg_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_mode_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double linear_step_ = 0.1;
    double lateral_step_ = 0.1;
    double angular_step_ = 0.1;
    double max_linear_x_ = 0.4;
    double max_linear_y_ = 0.3;
    double max_angular_z_ = 0.2;
    termios old_tio_{}, new_tio_{};
};


#endif //KEYBOARDINPUT_H
