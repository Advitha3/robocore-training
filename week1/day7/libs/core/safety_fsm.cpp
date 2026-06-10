#include "safety_fsm.hpp"
#include <iostream>

SafetyNode::SafetyNode()  : current_state_(SystemState::INIT){}

void SafetyNode::evaluate_health(int health_score){
    if(current_state_ == SystemState::EMERGENCY_STOP)return;

    if(health_score >= 100){
        current_state_ = SystemState::NORMAL_OP;
        } else if (health_score >= 50) {
            current_state_ = SystemState::DEGRADED;
        } else {
            current_state_ = SystemState::EMERGENCY_STOP;
        }
    }


void SafetyNode::manual_reset(){
    if (current_state_ == SystemState::EMERGENCY_STOP) {
        current_state_ = SystemState::INIT;
        std::cout << "[SAFETY FSM] Manual reset triggered.\n";
    } else {
        std::cout << "[SAFETY FSM] Reset not allowed.\n";
    }
}


SystemState SafetyNode :: get_state() const {return current_state_;}

void SafetyNode::print_state() const {
    std::cout << "[STATE] ";
    switch (current_state_) {
        case SystemState::INIT:           std::cout << "INIT\n"; break;
        case SystemState::NORMAL_OP:      std::cout << "NORMAL_OP\n"; break;
        case SystemState::DEGRADED:       std::cout << "DEGRADED\n"; break;
        case SystemState::EMERGENCY_STOP: std::cout << "EMERGENCY_STOP\n"; break;
    }
}