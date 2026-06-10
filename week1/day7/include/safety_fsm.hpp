#pragma once 
enum class SystemState{
    INIT,
    NORMAL_OP,
    DEGRADED,
    EMERGENCY_STOP
};

class SafetyNode{
private:
    SystemState current_state_;

public:
    SafetyNode();
    void evaluate_health(int health_score);
    void manual_reset();
    SystemState get_state() const;
    void print_state() const;
};