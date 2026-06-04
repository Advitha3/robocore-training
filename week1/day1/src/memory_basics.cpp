#include <iostream>
#include <cstring>
#include <cstddef>
#include <memory>
#include <queue> // Used to create the First-In-First-Out (FIFO) pipeline

// 1. Fixed-size Struct
struct SensorReading {
    char sensor_name[32];
    double value;
    double timestamp;

    SensorReading() : value(0.0), timestamp(0.0) {
        sensor_name[0] = '\0'; 
    }

    ~SensorReading() {
        // Only print destruction for the IMU to keep the console clean 
        if (std::strcmp(sensor_name, "Master_IMU") == 0) {
            std::cout << "!!! [Hardware] " << sensor_name << " completely destroyed !!!" << std::endl;
        }
    }

    void set_data(const char* name, double val, double time) {
        std::strncpy(sensor_name, name, sizeof(sensor_name) - 1);
        sensor_name[sizeof(sensor_name) - 1] = '\0';
        value = val;
        timestamp = time;
    }
};

// 2. Pool Allocator Class
class PoolAllocator {
public:
    static constexpr std::size_t POOL_SIZE = 8;
    static constexpr std::size_t INVALID = static_cast<std::size_t>(-1);

    PoolAllocator() : head(0), free_count(POOL_SIZE) {
        for (std::size_t i = 0; i < POOL_SIZE - 1; ++i) {
            free_list[i] = i + 1;
        }
        free_list[POOL_SIZE - 1] = INVALID;
    }

    SensorReading* allocate() {
        if (head == INVALID) return nullptr;
        std::size_t current_index = head;
        head = free_list[current_index];
        free_count--;
        return &pool[current_index];
    }

    void deallocate(SensorReading* ptr) {
        if (ptr == nullptr) return;
        std::size_t index = ptr - pool;
        if (index >= POOL_SIZE) return; // Safety bounds check
        
        free_list[index] = head;
        head = index;
        free_count++;
        std::cout << "   -> Returned slot " << index << " back to pool." << std::endl;
    }

    void status() const {
        std::size_t used_slots = POOL_SIZE - free_count;
        std::cout << "[Pool Status] Used: " << used_slots 
                  << " | Free: " << free_count << std::endl;
    }

private:
    SensorReading pool[POOL_SIZE];
    std::size_t free_list[POOL_SIZE];
    std::size_t head;
    std::size_t free_count;
};

// 3. Real-Time Pipeline Class
class RealTimeSensorPipeline {
private:
    PoolAllocator& allocator;
    std::queue<SensorReading*> processing_queue;

public:
    // Constructor requires a reference to the central allocator
    RealTimeSensorPipeline(PoolAllocator& alloc) : allocator(alloc) {}

    void push(const char* name, double val, double time) {
        SensorReading* new_reading = allocator.allocate();
        if (new_reading) {
            new_reading->set_data(name, val, time);
            processing_queue.push(new_reading);
            std::cout << "Pushed: " << name << " into pipeline." << std::endl;
        } else {
            std::cout << "Pipeline Error: Pool full, dropped " << name << std::endl;
        }
    }

    void pop() {
        if (processing_queue.empty()) return;

        // Grab the oldest reading from the front of the queue
        SensorReading* reading = processing_queue.front();
        processing_queue.pop();

        // Process it
        std::cout << "[Processing] Data from " << reading->sensor_name 
                  << " | Value: " << reading->value << std::endl;

        // Immediately return memory to the pool
        allocator.deallocate(reading);
    }

    bool is_empty() const {
        return processing_queue.empty();
    }
};

// 4. WatchDog Monitoring Function
void run_watchdog(const std::weak_ptr<SensorReading>& wd) {
    if (auto lock = wd.lock()) {
        std::cout << "[WatchDog] System nominal. IMU tracking at: " << lock->timestamp << std::endl;
    } else {
        std::cout << "⚠ [WatchDog] ALERT: IMU OFFLINE! " << std::endl;
    }
}

// 5. Main Simulation
int main() {
    std::cout << "===== INITIALIZING SYSTEM =====" << std::endl;
    PoolAllocator pool;
    RealTimeSensorPipeline pipeline(pool);

    // Create the Master IMU using smart pointers (Separate from the pool)
    std::shared_ptr<SensorReading> master_imu = std::make_shared<SensorReading>();
    master_imu->set_data("Master_IMU", 9.81, 100.001);
    
    // Register the watchdog to observe the IMU
    std::weak_ptr<SensorReading> watchdog = master_imu;

    std::cout << "\n===== FILLING PIPELINE =====" << std::endl;
    pipeline.push("Lidar_Front", 12.5, 200.1);
    pipeline.push("Lidar_Rear", 12.6, 200.2);
    pipeline.push("Camera_RGB", 30.0, 200.3);
    pipeline.push("Radar_Long", 80.0, 200.4);
    pipeline.push("Sonar_Short", 1.2, 200.5);

    std::cout << "\nStatus before processing:" << std::endl;
    pool.status(); // Should show Used: 5, Free: 3

    std::cout << "\n===== EXECUTING CONTROL LOOP =====" << std::endl;
    int cycle = 1;
    
    while (!pipeline.is_empty()) {
        std::cout << "\n--- Cycle " << cycle << " ---" << std::endl;
        
        // Mid-loop hardware failure simulation
        if (cycle == 3) {
            std::cout << ">> SIMULATING CATASTROPHIC CABLE DISCONNECT <<" << std::endl;
            master_imu.reset(); // Sensor goes offline
        }

        // 1. Diagnostics check
        run_watchdog(watchdog);
        
        // 2. Process next sensor in queue
        pipeline.pop();
        
        cycle++;
    }

    std::cout << "\n===== SYSTEM SHUTDOWN =====" << std::endl;
    std::cout << "Final Memory Map:" << std::endl;
    pool.status(); // Should show Used: 0, Free: 8

    return 0;
}