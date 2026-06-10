#pragma once 
#include <string>
#include <vector>

struct SensorReading{
    std::string sensor_name;
    double timestamp;
    bool is_healthy;
    std::vector<double> payload;
};

bool validate_sensor_health(const SensorReading& reading);