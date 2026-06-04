#include <iostream>
#include <string>
#include <memory>

struct SensorReading {
    std::string sensor_name;
    double value;
    double timestamp;

    SensorReading(std::string name, double val, double time)
        : sensor_name(name), value(val), timestamp(time) {}

    ~SensorReading() {
        std::cout << "!!! Destructor Fired: SensorReading freed !!!" << std::endl;
    }
};

int main() {
    // --- Task 2 Code (Kept intact for reference) ---
    {
        std::cout << "===== RUNNING TASK 2 =====" << std::endl;
        std::shared_ptr<SensorReading> main_ptr = std::make_shared<SensorReading>("IMU", 9.81, 16800056.7);
        std::cout << "After creation, use_count: " << main_ptr.use_count() << std::endl;

        std::shared_ptr<SensorReading> controller = main_ptr;
        std::cout << "After controller gets it, use_count: " << main_ptr.use_count() << std::endl;
        {
            std::cout << "\n--- Entering Safety Monitor Scope ---" << std::endl;
            std::shared_ptr<SensorReading> safety_monitor = main_ptr;
            std::cout << "After safety gets it, use_count: " << main_ptr.use_count() << std::endl;
            std::cout << "Safety tracking sensor: " << safety_monitor->sensor_name << std::endl;
            std::cout << "--- Leaving Safety Monitor Scope ---" << std::endl;
        }
        std::cout << "After safety scope ends, use_count: " << main_ptr.use_count() << std::endl;
        std::cout << "===== TASK 2 ENDING (Cleaning memory) =====\n" << std::endl;
    } // All Task 2 memory is cleared here because brackets closed.

    // --- Task 3: Weak Pointer Implementation ---
    std::cout << "===== STARTING TASK 3 =====" << std::endl;

    // 1. Create a fresh SensorReading with a shared_ptr
    std::shared_ptr<SensorReading> shared_sensor = std::make_shared<SensorReading>("GPS", 45.1234, 168000999.0);
    std::cout << "use_count after shared_ptr created: " << shared_sensor.use_count() << std::endl; // Should be 1

    // 2. Create a weak_ptr pointing to the shared_ptr
    std::weak_ptr<SensorReading> weak_observer = shared_sensor;
    std::cout << "use_count after weak_ptr created: " << shared_sensor.use_count() << std::endl; // Should STILL be 1

    // 3. Simulate the sensor going offline by resetting the shared_ptr
    std::cout << "\n--- Simulating Sensor Disconnection (.reset()) ---" << std::endl;
    shared_sensor.reset(); // This destroys the object because use_count hits 0.
    // You will see the destructor print immediately right here!

    // 4. Try to access the reading through weak_ptr after shared_ptr is gone
    std::cout << "\nAttempting to read data via weak_ptr..." << std::endl;
    
    // Use .lock() to safely check if the memory is still valid
    if (std::shared_ptr<SensorReading> locked_ptr = weak_observer.lock()) {
        // This block will NOT execute because the sensor is already gone
        std::cout << "Sensor is Online! Value: " << locked_ptr->value << std::endl;
    } else {
        // This block handles the offline status gracefully without crashing the program!
        std::cout << "Graceful Handle: Sensor is OFFLINE. Cannot access memory." << std::endl;
    }

    std::cout << "===== TASK 3 ENDING =====" << std::endl;
    return 0;
}