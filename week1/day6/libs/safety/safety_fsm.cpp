#include "safety_fsm.hpp"
#include <iostream>

SafetyNode::SafetyNode() : current_state_(SystemState::INIT) {}

void SafetyNode::evaluate_health(int health_score) {
    // HARD LATCH: If we are in E-STOP, ignore all automatic health updates.
    // The system MUST be manually reset to exit this state.
    if (current_state_ == SystemState::EMERGENCY_STOP) {
        return; 
    }

    // FSM Transition Logic
    if (health_score >= 100) {
        current_state_ = SystemState::NORMAL_OP;
    } 
    else if (health_score >= 50) {
        current_state_ = SystemState::DEGRADED;
    } 
    else {
        current_state_ = SystemState::EMERGENCY_STOP;
    }
}

void SafetyNode::manual_reset() {
    // Only allow reset if the system is actually in an E-STOP condition
    if (current_state_ == SystemState::EMERGENCY_STOP) {
        current_state_ = SystemState::INIT;
        std::cout << "[SAFETY FSM] Manual reset triggered. Returning to INIT state.\n";
    } else {
        std::cout << "[SAFETY FSM] Reset not allowed in current state.\n";
    }
}

SystemState SafetyNode::get_state() const { return current_state_; }

void SafetyNode::print_state() const {
    std::cout << "[SAFETY FSM] Current State: ";
    switch (current_state_) {
        case SystemState::INIT:           std::cout << "INIT\n"; break;
        case SystemState::NORMAL_OP:      std::cout << "NORMAL_OP\n"; break;
        case SystemState::DEGRADED:       std::cout << "DEGRADED (Partial Failure)\n"; break;
        case SystemState::EMERGENCY_STOP: std::cout << "EMERGENCY_STOP (E-STOP)\n"; break;
    }
}