#include <iostream>
#include <cstring> // Required for std::strncpy
#include <cstddef> // Required for std::size_t
#include <memory>
#include <queue>

// 1. Fixed-size Struct (No hidden heap allocations)
struct SensorReading {
    char sensor_name[32]; // Fixed size, lives completely inside the struct
    double value;
    double timestamp;

    // Default constructor needed to initialize the array in PoolAllocator
    SensorReading() : value(0.0), timestamp(0.0) {
        sensor_name[0] = '\0'; 
    }

    ~SensorReading(){
        if(std::strcmp(sensor_name, "Master_IMU")==0){
            std::cout <<"!!![Hardware]"<<sensor_name<<"Completely destroyed !!!"<<std::endl;

    
        }
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
                  << " | Free: " << free_count << std::endl;
    }

private:
    SensorReading pool[POOL_SIZE];       
    std::size_t free_list[POOL_SIZE];    
    std::size_t head;                    
    std::size_t free_count;              
};

class RealTimeSensorPIpeline{
private:
    PoolAllocator& allocator;
    std::queue<SensorReading*> processing_queue;

public:
    RealTimeSensorPIpeline(PoolAllocator& alloc): allocator(alloc){}

    void push(const char* name, double val, double time){
        SensorReading* new_reading = allocator.allocate();
        if(new_reading){
            new_reading->set_data(name,val,time);
            processing_queue.push(new_reading);
            std::cout <<"Pushed:" <<name << "into pipeline" <<std::endl;
        }else{
            std::cout<<" Error"<<name<<std::endl;
        }
    }

    void pop(){
        if(processing_queue.empty())return;

        SensorReading* reading = processing_queue.front();
        processing_queue.pop();

        std::cout <<"[Processing ] data from" <<reading->sensor_name
                  <<"|value:" <<reading->value <<std::endl;

        allocator.deallocate(reading);
    }

    bool is_empty() const {
        return processing_queue.empty();
    }
};

void run_watchdog(const std::weak_ptr<SensorReading>& wd){
    if(auto lock = wd.lock()){
        std::cout <<"[WatchDog]System nominal. IMU tracking at :" << lock->timestamp << std ::endl;
    }else {
        std::cout<<" [WatchDog] ALERT: IMU OFFLINE" <<std::endl;
    }
}
int main() {
    std::cout <<"===INITIALIZING SYSTEM ===" <<std::endl;
    PoolAllocator pool;
    RealTimeSensorPIpeline pipeline(pool);

    std::shared_ptr<SensorReading> master_imu = std::make_shared<SensorReading>();
    master_imu->set_data("Master_IMU", 9.81, 100.001);

    std::weak_ptr<SensorReading> watchdog = master_imu;

    std::cout << "\n===== FILLING PIPELINE =====" << std::endl;
    pipeline.push("Lidar_Front", 12.5, 200.1);
    pipeline.push("Lidar_Rear", 12.6, 200.2);
    pipeline.push("Camera_RGB", 30.0, 200.3);
    pipeline.push("Radar_Long", 80.0, 200.4);
    pipeline.push("Sonar_Short", 1.2, 200.5);

    std::cout <<"\nStatus before processing:" <<std::endl;
    pool.status(); // shou;d show used :5 , free:3 

    std::cout <<"\n ====EXECUTING CONTROL LOOP===" << std::endl;
    int cycle =1;

    while (!pipeline.is_empty()){
        std::cout <<"/n---cycle" << cycle << "---" << std::endl;

        //mid-loop hardware failure simulatiuon 

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