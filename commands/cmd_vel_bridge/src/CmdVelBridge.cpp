#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <std_msgs/msg/int32.hpp>
#include <algorithm>

using namespace std::chrono_literals;

class CmdVelBridge : public rclcpp::Node {
public:
    CmdVelBridge() : Node("cmd_vel_bridge") {
        // 参数配置
        this->declare_parameter("max_vx", 0.4);
        this->declare_parameter("max_vy", 0.3);
        this->declare_parameter("max_wz", 0.2);
        this->declare_parameter("watchdog_timeout", 0.2);

        this->get_parameter("max_vx", max_vx_);
        this->get_parameter("max_vy", max_vy_);
        this->get_parameter("max_wz", max_wz_);
        this->get_parameter("watchdog_timeout", watchdog_timeout_);

        // 订阅 cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelBridge::cmdVelCallback, this, std::placeholders::_1));

        // 订阅模式切换指令
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "cmd_mode", 10, std::bind(&CmdVelBridge::modeCallback, this, std::placeholders::_1));

        // 发布控制器指令
        input_pub_ = this->create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);

        // --- 关键修复：初始化时间戳 ---
        // 这会将 last_cmd_vel_time_ 的时钟类型设置为与节点一致 (例如 ROS_TIME)
        last_cmd_vel_time_ = this->now(); 

        // 50Hz 定时器
        timer_ = this->create_wall_timer(20ms, std::bind(&CmdVelBridge::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "CmdVelBridge initialized successfully.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_time_ = this->now();
        
        // 坐标系转换逻辑
        target_inputs_.ly = std::clamp(msg->linear.x / max_vx_, -1.0, 1.0);
        target_inputs_.lx = std::clamp(-msg->linear.y / max_vy_, -1.0, 1.0);
        target_inputs_.rx = std::clamp(-msg->angular.z / max_wz_, -1.0, 1.0);
        target_inputs_.ry = 0.0;
    }

    void modeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        pending_command_ = msg->data;
        command_frames_left_ = 10;
        RCLCPP_INFO(this->get_logger(), "Switching mode to: %d", msg->data);
    }

    void timerCallback() {
        auto msg = control_input_msgs::msg::Inputs();
        
        // 看门狗检查 (Time subtraction is now safe)
        try {
            auto time_diff = (this->now() - last_cmd_vel_time_).seconds();
            if (time_diff > watchdog_timeout_) {
                msg.lx = 0.0; msg.ly = 0.0; msg.rx = 0.0; msg.ry = 0.0;
            } else {
                msg.lx = target_inputs_.lx;
                msg.ly = target_inputs_.ly;
                msg.rx = target_inputs_.rx;
                msg.ry = target_inputs_.ry;
            }
        } catch (const std::runtime_error& e) {
            // 防止极端情况下的崩溃，重新同步时间
            last_cmd_vel_time_ = this->now();
            return;
        }

        // 处理模式切换命令
        if (command_frames_left_ > 0) {
            msg.command = pending_command_;
            command_frames_left_--;
        } else {
            msg.command = 0;
        }
        input_pub_->publish(msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr input_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double max_vx_, max_vy_, max_wz_, watchdog_timeout_;
    rclcpp::Time last_cmd_vel_time_;
    control_input_msgs::msg::Inputs target_inputs_;
    int pending_command_ = 0;
    int command_frames_left_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelBridge>());
    rclcpp::shutdown();
    return 0;
}