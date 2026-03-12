//
// Created by biao on 24-9-10.
//

#include "unitree_guide_controller/FSM/StateFixedStand.h"

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd) {
}

FSMStateName StateFixedStand::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.motion_command_.requested_state_) {
        case FSMStateName::PASSIVE:
            return FSMStateName::PASSIVE;
        case FSMStateName::FIXEDDOWN:
            return FSMStateName::FIXEDDOWN;
        case FSMStateName::FREESTAND:
            return FSMStateName::FREESTAND;
        case FSMStateName::TROTTING:
            return FSMStateName::TROTTING;
        case FSMStateName::SWINGTEST:
            return FSMStateName::SWINGTEST;
        case FSMStateName::BALANCETEST:
            return FSMStateName::BALANCETEST;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
