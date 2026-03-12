//
// Created by biao on 24-9-11.
//

#include <algorithm>

#include <keyboard_input/KeyboardInput.h>

KeyboardInput::KeyboardInput() : Node("keyboard_input_node")
{
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    robot_mode_publisher_ = create_publisher<std_msgs::msg::Int32>("/robot_mode", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&KeyboardInput::timer_callback, this));

    declare_parameter("linear_step", linear_step_);
    declare_parameter("lateral_step", lateral_step_);
    declare_parameter("angular_step", angular_step_);
    declare_parameter("max_linear_x", max_linear_x_);
    declare_parameter("max_linear_y", max_linear_y_);
    declare_parameter("max_angular_z", max_angular_z_);

    get_parameter("linear_step", linear_step_);
    get_parameter("lateral_step", lateral_step_);
    get_parameter("angular_step", angular_step_);
    get_parameter("max_linear_x", max_linear_x_);
    get_parameter("max_linear_y", max_linear_y_);
    get_parameter("max_angular_z", max_angular_z_);

    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);

    RCLCPP_INFO(get_logger(), "Keyboard teleop started.");
    RCLCPP_INFO(get_logger(), "Modes: 1 passive, 2 fixeddown, 3 fixedstand, 4 trotting.");
    RCLCPP_INFO(get_logger(), "Motion: W/S forward, A/D lateral, Q/E yaw, space stop.");
}

void KeyboardInput::timer_callback()
{
    if (kbhit())
    {
        handle_key(static_cast<char>(getchar()));
    }

    cmd_vel_publisher_->publish(twist_msg_);
}

void KeyboardInput::handle_key(const char key)
{
    switch (key)
    {
    case '1':
    case '2':
    case '3':
    case '4':
        publish_mode(key - '0');
        return;
    case 'w':
    case 'W':
        twist_msg_.linear.x = std::clamp(twist_msg_.linear.x + linear_step_, -max_linear_x_, max_linear_x_);
        break;
    case 's':
    case 'S':
        twist_msg_.linear.x = std::clamp(twist_msg_.linear.x - linear_step_, -max_linear_x_, max_linear_x_);
        break;
    case 'a':
    case 'A':
        twist_msg_.linear.y = std::clamp(twist_msg_.linear.y + lateral_step_, -max_linear_y_, max_linear_y_);
        break;
    case 'd':
    case 'D':
        twist_msg_.linear.y = std::clamp(twist_msg_.linear.y - lateral_step_, -max_linear_y_, max_linear_y_);
        break;
    case 'q':
    case 'Q':
        twist_msg_.angular.z = std::clamp(twist_msg_.angular.z + angular_step_, -max_angular_z_, max_angular_z_);
        break;
    case 'e':
    case 'E':
        twist_msg_.angular.z = std::clamp(twist_msg_.angular.z - angular_step_, -max_angular_z_, max_angular_z_);
        break;
    case ' ':
        stop_motion();
        break;
    default:
        break;
    }
}

void KeyboardInput::publish_mode(const int mode)
{
    std_msgs::msg::Int32 mode_msg;
    mode_msg.data = mode;
    robot_mode_publisher_->publish(mode_msg);
    RCLCPP_INFO(get_logger(), "Requested robot mode: %d", mode);
}

void KeyboardInput::stop_motion()
{
    twist_msg_ = geometry_msgs::msg::Twist();
    RCLCPP_INFO(get_logger(), "Motion command reset.");
}

bool KeyboardInput::kbhit()
{
    timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
