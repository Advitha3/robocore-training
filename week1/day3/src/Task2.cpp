#include<iostream>
#include<cstddef>
#include <cstring> // For std::memcpy
#include <iomanip> // For std::hex and std::setprecision

template <typename T, std::size_t SIZE>
class CircularBuffer{
private:
    T buffer_[SIZE];
    std::size_t read_idx_= 0;
    std::size_t write_idx_= 0;
    std::size_t count_=0;

public:
    bool push( const T& item){
        if (is_full()){ 
        return false;
    }

    buffer_[write_idx_] = item;
    //the modulo wrap :if write_idx_ hits SIZE, it loops back to 0 

    write_idx_= (write_idx_ +1)%SIZE;
    count_++;
    return true;
}

    bool pop(T& out ){
        if (is_empty()){
        return false;
    }

    out = buffer_[read_idx_];

    //modulo wrap applies to the read index as well 
    read_idx_ = (read_idx_ +1)% SIZE;
    count_--;
    return true ;
}

    bool is_full() const {return count_ == SIZE;}
    bool is_empty() const{return count_ ==0;}
    std::size_t size() const{return count_;}
};

struct CANMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    double timestamp;
    
    void print() const{
        std::cout << "[CAN Msg] Time: " << std::fixed << std::setprecision(3) << timestamp
                  << " | ID: 0x" << std::hex << id << std::dec // switch to hex, then back to decimal
                  << " | Len: " << (int)length 
                  << " | Data: ";

        for(int i =0; i < length; ++i){
            std::cout <<(char)data[i];
        }
        std::cout << std::endl;
    }
};

int main() {
    std::cout << "===== INITIALIZING CAN BUS PIPELINE =====" << std::endl;
    
    // Create the buffer specifically for CANMessages, size 16
    CircularBuffer<CANMessage, 16> can_bus;
    CANMessage msg; // A temporary struct to load our data into

    std::cout << "\n>>> SIMULATING HARDWARE SENSOR INPUTS..." << std::endl;

    // 1. Push IMU Data
    msg.id = 0x100;
    std::memcpy(msg.data, "IMU", 3);
    msg.length = 3;
    msg.timestamp = 1.001;
    can_bus.push(msg);
    std::cout << "Pushed IMU message to buffer." << std::endl;

    // 2. Push Encoder Data
    msg.id = 0x200;
    std::memcpy(msg.data, "ENC", 3);
    msg.length = 3;
    msg.timestamp = 1.002;
    can_bus.push(msg);
    std::cout << "Pushed Encoder message to buffer." << std::endl;

    // 3. Push Lidar Data
    msg.id = 0x300;
    std::memcpy(msg.data, "LDR", 3);
    msg.length = 3;
    msg.timestamp = 1.003;
    can_bus.push(msg);
    std::cout << "Pushed Lidar message to buffer." << std::endl;

    // 4. Push Camera Data
    msg.id = 0x400;
    std::memcpy(msg.data, "CAM", 3);
    msg.length = 3;
    msg.timestamp = 1.004;
    can_bus.push(msg);
    std::cout << "Pushed Camera message to buffer." << std::endl;

    std::cout << "\n>>> PROCESSING HARDWARE QUEUE (POP)..." << std::endl;

    // Pop and print all messages until the buffer is empty
    CANMessage out_msg;
    while (!can_bus.is_empty()) {
        can_bus.pop(out_msg);
        out_msg.print();
    }

    std::cout << "\n===== PIPELINE CLEAR =====" << std::endl;

    return 0;
}
