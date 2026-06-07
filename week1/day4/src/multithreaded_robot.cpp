#include<iostream>
#include<thread>
#include<atomic>



std::atomic<int>sensor_count (0);

void increment(){
    for(int i = 0 ;i < 1000; i++ )
    {
        
        sensor_count.fetch_add(1);
    }
}

int main(){
    std::thread IMU(increment);
    std::thread Lidar(increment);
    std::thread Camera(increment);

    IMU.join();
    Lidar.join();
    Camera.join();

    std::cout<<"Final count:"<< sensor_count << std::endl;
}