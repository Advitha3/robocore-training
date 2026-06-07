#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <iomanip>
#include<atomic>



struct RobotState{
    double accel;
    double distance;
    double velocity;
    double last_update;
};

RobotState state;
std::mutex state_mutex;
std::atomic<bool> running{true};// Atomic: Thread-safe kill switch without needing a lock

// Helper function to get a clean timestamp
double get_timestamp(){
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

void imu_thread(){
    while (running){
    {
        std::lock_guard<std::mutex >lock(state_mutex);
        state.accel = 9.81;
        state.last_update = get_timestamp();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
}

void lidar_thread(){
    while(running){
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state.distance = 3.45;
            state.last_update = get_timestamp();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void encoder_thread(){
    while(running){
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state.velocity = 1.2;
            state.last_update = get_timestamp(); 
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
}

void fusion_thread(){
    int cycles = 0;
    while(running && cycles <20){
        RobotState local_state;

        // 1. Lock briefly just to copy the state
        {
            std::lock_guard<std::mutex>lock(state_mutex);
            local_state = state;
        }
        // We unlock instantly so the sensors can keep writing!
        std::cout << "[FUSED] " 
                  << "accel:" << std::fixed << std::setprecision(2) << local_state.accel 
                  << " | dist:" << local_state.distance 
                  << " | vel:" << local_state.velocity 
                  << " | t:" << std::setprecision(6) << local_state.last_update 
                  << std::endl;

        cycles++;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    }

    running = false;
}

int main(){
    std::cout << "===== IGNITING SENSOR THREADS =====" <<std::endl;

    std::thread t_imu(imu_thread);
    std::thread t_lidar(lidar_thread);
    std:: thread t_enc(encoder_thread);
    std::thread t_fusion(fusion_thread);

    t_fusion.join();
    t_imu.join();
    t_lidar.join();
    t_enc.join();

    std::cout << "===== ALL THREADS JOINED CLEANLY. SHUTDOWN COMPLETE. =====" << std::endl;

    return 0;


}








