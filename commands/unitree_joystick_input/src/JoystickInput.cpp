//
// Created by tlab-uav on 24-9-13.
//

#include "unitree_joystick_input/JoystickInput.h"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using std::placeholders::_1;

using namespace unitree::robot;

JoystickInput::JoystickInput() : Node("unitree_joysick_node")
{
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    robot_mode_publisher_ = create_publisher<std_msgs::msg::Int32>("/robot_mode", 10);

    declare_parameter("network_interface", network_interface_);
    declare_parameter("domain", domain_);

    network_interface_ = get_parameter("network_interface").as_string();
    domain_ = get_parameter("domain").as_int();
    RCLCPP_INFO(get_logger(), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(
        TOPIC_JOYSTICK);
    subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            remoteWirelessHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);
}

void JoystickInput::remoteWirelessHandle(const void* messages)
{
    wireless_controller_ = *static_cast<const unitree_go::msg::dds_::WirelessController_*>(messages);
    xKeySwitchUnion_.value = wireless_controller_.keys();

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.4 * wireless_controller_.ly();
    twist_msg.linear.y = -0.3 * wireless_controller_.lx();
    twist_msg.angular.z = -0.2 * wireless_controller_.rx();
    cmd_vel_publisher_->publish(twist_msg);

    std_msgs::msg::Int32 mode_msg;
    if (xKeySwitchUnion_.components.select)
    {
        mode_msg.data = 1;
    }
    else if (xKeySwitchUnion_.components.start)
    {
        mode_msg.data = 2;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.X)
    {
        mode_msg.data = 3;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.Y)
    {
        mode_msg.data = 4;
    }
    else
    {
        return;
    }
    robot_mode_publisher_->publish(mode_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
