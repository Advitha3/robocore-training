#include "pool_allocator.hpp"
#include "circular_buffer.hpp"
#include "safety_fsm.hpp"
#include "sensor_reading.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

// Global components shared between threads
PoolAllocator<SensorReading, 16> pool;
CircularBuffer<SensorReading*, 16> buffer;
SafetyNode fsm;

// Thread synchronization primitives
std::mutex pool_mutex;
std::mutex buffer_mutex;
std::mutex fsm_mutex;
std::atomic<bool> is_running{true};

// --- THREAD 1: SENSOR PRODUCER ---
void sensor_producer_thread() {
    int loop_count = 0;
    while (is_running && loop_count < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        SensorReading* reading = nullptr;
        
        // Protected allocation from the pool
        {
            std::lock_guard<std::mutex> lock(pool_mutex);
            reading = pool.allocate();
        }

        if (reading == nullptr) {
            std::cout << "[PRODUCER] Pool full! Dropping frame.\n";
            continue;
        }

        // Populating sensor data block
        reading->sensor_name = "IMU_Primary";
        reading->timestamp = loop_count * 0.2;
        
        if (loop_count == 3) {
            reading->is_healthy = false; // Trigger failure state on 4th cycle
            std::cout << "[PRODUCER] Hardware Error Dispatched!\n";
        } else {
            reading->is_healthy = true;
        }

        // Protected push to the data buffer
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            buffer.push(reading);
        }
        std::cout << "[PRODUCER] Sent frame " << loop_count << "\n";
        loop_count++;
    }
}

// --- THREAD 2: SAFETY CONSUMER ---
void safety_consumer_thread() {
    while (is_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        SensorReading* reading = nullptr;
        bool has_data = false;

        // Protected pop from the data buffer
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            has_data = buffer.pop(reading);
        }

        if (has_data && reading != nullptr) {
            std::cout << "[CONSUMER] Processing data from: " << reading->sensor_name << "\n";
            
            int health_score = reading->is_healthy ? 100 : 30;
            
            // Protected modification of the FSM state
            {
                std::lock_guard<std::mutex> lock(fsm_mutex);
                fsm.evaluate_health(health_score);
                fsm.print_state();
            }

            // Protected deallocation back to the pool
            {
                std::lock_guard<std::mutex> lock(pool_mutex);
                pool.deallocate(reading);
            }
        }
    }
}

// --- THREAD 3: SAFETY WATCHDOG ---
void watchdog_thread() {
    while (is_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Protected read-only access to verify system safety state
        {
            std::lock_guard<std::mutex> lock(fsm_mutex);
            if (fsm.get_state() == SystemState::EMERGENCY_STOP) {
                std::cout << "[WATCHDOG] EMERGENCY_STOP detected! Initiating immediate system shutdown.\n";
                is_running = false; 
                return;
            }
        }
    }
}

int main() {
    std::cout << "===== STARTING SYNCHRONIZED MULTI-THREADED ROBOT CORE =====\n";
    {
        std::lock_guard<std::mutex> lock(fsm_mutex);
        fsm.print_state();
    }

    // Spawn all execution loops
    std::thread producer(sensor_producer_thread);
    std::thread consumer(safety_consumer_thread);
    std::thread watchdog(watchdog_thread);

    // Wait for systems to settle and terminate cleanly
    producer.join();
    consumer.join();
    watchdog.join();

    std::cout << "===== SHUTTING DOWN ROBOT CORE CLEANLY =====\n";
    return 0;
}