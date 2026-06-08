#include <iostream>
#include <vector>
#include <cstring>
#include <utility>


class MoveableSensorReading {
public:
    std::vector<double> raw_data;
    char sensor_name[32];
    double timestamp;

    // Standard Constructor
    MoveableSensorReading(const char* name, double ts, size_t points) {
        std::strncpy(sensor_name, name, sizeof(sensor_name) - 1);
        sensor_name[sizeof(sensor_name) - 1] = '\0';
        timestamp = ts;
        raw_data = std::vector<double>(points, 0.0);
    }

    // Copy Constructor
    MoveableSensorReading(const MoveableSensorReading& other) {
        raw_data = other.raw_data; // O(N) physically copies all elements
        std::strncpy(sensor_name, other.sensor_name, sizeof(sensor_name));
        timestamp = other.timestamp;
        std::cout << "[COPY] COPY — expensive! Copied " << raw_data.size() << " points." << std::endl;
    }

    // Move Constructor
    MoveableSensorReading(MoveableSensorReading&& other) noexcept {
        raw_data = std::move(other.raw_data); // O(1) steals the memory pointers
        std::strncpy(sensor_name, other.sensor_name, sizeof(sensor_name));
        timestamp = other.timestamp;
        
        // Nullify 
        other.sensor_name[0] = '\0'; 
        other.timestamp = 0.0;

        std::cout << "[MOVE] MOVE — free! Stole " << raw_data.size() << " points." << std::endl;
    }
};


class SensorQueue {
private:
    std::vector<MoveableSensorReading> queue_;

public:
    SensorQueue() {
        // PRO TIP: Reserve space upfront so the vector doesn't resize itself
        // behind the scenes and trigger unwanted copy/move constructors.
        queue_.reserve(10); 
    }

    // Accepts L-Values (standard variables) -> Triggers Copy
    void push_copy(const MoveableSensorReading& r) {
        queue_.push_back(r); 
    }

    // Accepts R-Values (temporary or explicitly moved variables) -> Triggers Move
    void push_move(MoveableSensorReading&& r) {
        // We must call std::move(r) again here to pass the r-value nature 
        // down into the push_back function.
        queue_.push_back(std::move(r)); 
    }

    size_t size() const {
        return queue_.size();
    }
};


int main() {
    SensorQueue queue;

    std::cout << "1. Creating LiDAR reading 1 (500 points)..." << std::endl;
    MoveableSensorReading scan1("LiDAR_Front", 1.001, 500);

    std::cout << "\n2. Pushing by COPY..." << std::endl;
    queue.push_copy(scan1);

    std::cout << "\n3. Creating LiDAR reading 2 (500 points)..." << std::endl;
    MoveableSensorReading scan2("LiDAR_Rear", 1.002, 500);

    std::cout << "\n4. Pushing by MOVE..." << std::endl;
    // We explicitly cast scan2 to an r-value to trigger push_move
    queue.push_move(std::move(scan2)); 

    std::cout << "\n5. Checking Queue Size..." << std::endl;
    std::cout << "   Total elements in queue: " << queue.size() << std::endl;

    std::cout << "\n6. Checking original variables..." << std::endl;
    std::cout << "   scan1 (Copied) size: " << scan1.raw_data.size() << " points (Still intact)" << std::endl;
    std::cout << "   scan2 (Moved) size:  " << scan2.raw_data.size() << " points (Empty!)" << std::endl;

    return 0;
}