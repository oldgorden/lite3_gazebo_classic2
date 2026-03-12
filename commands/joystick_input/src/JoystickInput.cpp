//
// Created by tlab-uav on 24-9-13.
//

#include "joystick_input/JoystickInput.h"

using std::placeholders::_1;

JoystickInput::JoystickInput() : Node("joysick_input_node") {
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    robot_mode_publisher_ = create_publisher<std_msgs::msg::Int32>("/robot_mode", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickInput::joy_callback, this, _1));
}

void JoystickInput::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.4 * msg->axes[1];
    twist_msg.linear.y = -0.3 * msg->axes[0];
    twist_msg.angular.z = -0.2 * msg->axes[3];
    cmd_vel_publisher_->publish(twist_msg);

    std_msgs::msg::Int32 mode_msg;
    if (msg->buttons[1] && msg->buttons[4]) {
        mode_msg.data = 1; // LB + B -> passive
    } else if (msg->buttons[0] && msg->buttons[4]) {
        mode_msg.data = 2; // LB + A -> fixeddown
    } else if (msg->buttons[2] && msg->buttons[4]) {
        mode_msg.data = 3; // LB + X -> fixedstand
    } else if (msg->buttons[3] && msg->buttons[4]) {
        mode_msg.data = 4; // LB + Y -> trotting
    } else {
        return;
    }
    robot_mode_publisher_->publish(mode_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
