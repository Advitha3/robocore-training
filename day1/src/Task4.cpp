#include <iostream>
#include <cstring> // Required for std::strncpy
#include <cstddef> // Required for std::size_t

// 1. Fixed-size Struct (No hidden heap allocations)
struct SensorReading {
    char sensor_name[32]; // Fixed size, lives completely inside the struct
    double value;
    double timestamp;

    // Default constructor needed to initialize the array in PoolAllocator
    SensorReading() : value(0.0), timestamp(0.0) {
        sensor_name[0] = '\0'; 
    }

    // Helper method to easily set data
    void set_data(const char* name, double val, double time) {
        // Safely copy string into fixed buffer ensuring null-termination
        std::strncpy(sensor_name, name, sizeof(sensor_name) - 1);
        sensor_name[sizeof(sensor_name) - 1] = '\0';
        value = val;
        timestamp = time;
    }
};

// 2. Pool Allocator Class
class PoolAllocator {
public:
    // --- NEW: Helper method to safely get the index ---
    std::size_t get_index(const SensorReading* ptr) const {
        return ptr - pool; // This works here because it is inside the class!
    }
    static constexpr std::size_t POOL_SIZE = 8;
    static constexpr std::size_t INVALID = static_cast<std::size_t>(-1);

    // 3. Constructor
    PoolAllocator() : head(0), free_count(POOL_SIZE) {
        // Build the free list: each slot points to the next available slot index
        for (std::size_t i = 0; i < POOL_SIZE - 1; ++i) {
            free_list[i] = i + 1;
        }
        free_list[POOL_SIZE - 1] = INVALID; // Last entry terminates the list
    }

        // Print reserved bytes
    SensorReading* allocate(){
        if(head == INVALID){
            std::cout<<"[ERROR]Allocation failed poll is full!" <<std::endl;
            return nullptr;
        }

        std::size_t current_index = head;
        head = free_list[current_index];
        free_count--;
        return &pool[current_index];
    }

    void deallocate(SensorReading* ptr){
        if(ptr == nullptr) return;

        std::size_t index = ptr - pool;

        if(index >= POOL_SIZE){
            std::cout <<"[ERROR]Deallocation failed:Pointer is out of poll bounds"<<std::endl;
            return;
        }
        
        free_list[index] = head;
        head = index;
        free_count++;
        std::cout <<"Successfully freed slot index" <<index <<std::endl;

    }

    // 4. Status Method
    void status() const {
        std::size_t used_slots = POOL_SIZE - free_count;
        std::cout << "[Pool Status] Used: " << used_slots 
                  << " | Free: " << free_count 
                  << " | Next Available Index (Head): ";
        if (head == INVALID) {
            std::cout << "NONE (Pool Full)" << std::endl;
        } else {
            std::cout << head << std::endl;
        }
    }

private:
    SensorReading pool[POOL_SIZE];       // The actual block of memory
    std::size_t free_list[POOL_SIZE];    // Tracks the index of the next free slot
    std::size_t head;                    // Points to the first available free slot index
    std::size_t free_count;              // Tracking variable for diagnostics
};

int main() {
    // Instantiate the pool to trigger the constructor and check status
    PoolAllocator allocator;
    allocator.status();

    std::cout <<"\n----Allocating 2 sensors---"<<std::endl;
    SensorReading* s1 = allocator.allocate();
    s1->set_data("Lidar_front", 15.5, 100.1);

    SensorReading* s2 = allocator.allocate();
    s2 ->set_data("camera_left",39.0,100.2);

    allocator.status();

    std::cout <<"\n---Delloocate 1st sensor ---"<< std::endl;
    allocator.deallocate(s1);
    allocator.status();

    std::cout << "\n--- Allocating another Sensor ---" << std::endl;
    SensorReading* s3 = allocator.allocate();
    s3->set_data("Radar_Rear", 5.0, 100.3);
   // Change this line:
    // std::cout << "New sensor placed at index: " << (s3 - allocator.pool) << std::endl;
    
    // To this:
    std::cout << "New sensor placed at index: " << allocator.get_index(s3) << std::endl;
    
    allocator.status();

    return 0;
}