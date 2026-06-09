#include "sensor_types.hpp"
#include <iostream>

// Implementation of the validation logic
bool validate_sensor_health(const SensorReading& reading) {
    if (!reading.is_healthy || reading.payload.empty()) {
        std::cout << "[WARN] Sensor Failed: " << reading.sensor_name << std::endl;
        return false;
    }
    return true;
}