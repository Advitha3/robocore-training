#include "safety_fsm.hpp"
#include <iostream>

int main() {
    std::cout << "===== BOOTING ROBOT CORE =====\n";
    SafetyNode safety_manager;
    safety_manager.print_state();

    std::cout << "\n--- Boot (Score 100) ---\n";
    safety_manager.evaluate_health(100);
    safety_manager.print_state();

    std::cout << "\n--- IMU Fails (Score 60) ---\n";
    safety_manager.evaluate_health(60);
    safety_manager.print_state();

    std::cout << "\n--- LiDAR Fails (Score 30) ---\n";
    safety_manager.evaluate_health(30);
    safety_manager.print_state();

    std::cout << "\n--- Attempt Manual Reset ---\n";
    safety_manager.manual_reset();
    safety_manager.print_state();

    std::cout << "\n--- Attempt Reset Again ---\n";
    safety_manager.manual_reset();

    return 0;
}