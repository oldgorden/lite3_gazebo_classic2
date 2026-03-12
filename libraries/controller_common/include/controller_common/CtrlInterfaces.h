//
// Created by biao on 24-9-10.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <controller_common/common/enumClass.h>

struct MotionCommand
{
    FSMStateName requested_state_{FSMStateName::PASSIVE};
    double linear_x_{0.0};
    double linear_y_{0.0};
    double angular_z_{0.0};
    bool cmd_vel_active_{false};

    void reset_twist()
    {
        linear_x_ = 0.0;
        linear_y_ = 0.0;
        angular_z_ = 0.0;
        cmd_vel_active_ = false;
    }
};

struct CtrlInterfaces
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kd_command_interface_;


    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    imu_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    foot_force_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    odom_state_interface_;
    MotionCommand motion_command_;
    int frequency_{};

    CtrlInterfaces() = default;

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        imu_state_interface_.clear();
        foot_force_state_interface_.clear();
    }
};

#endif //INTERFACE_H
