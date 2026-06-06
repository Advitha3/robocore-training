#include <iostream>
#include <cstddef>
#include <cstring>
#include <iomanip>

struct CANMessage{
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    double timestamp;

    void print() const {
        std::cout << "[CAN] Time: " << std::fixed << std::setprecision(3) << timestamp
                  << " | ID: 0x" << std::hex << id << std::dec
                  << " | Len: " << (int)length << " | Data: ";
        for (int i = 0; i < length; ++i) {
            std::cout << (char)data[i];
        }
        std::cout << std::endl;
    }
};

template <typename T, std::size_t SIZE>
class PoolAllocator{
private:
    T pool_[SIZE];
    bool free_map_[SIZE];
    std::size_t used_count_=0;

public:
PoolAllocator(){
    for(std::size_t i=0;i<SIZE;++i){
        free_map_[i] =true;
    }
}

T* allocate(){
    if(used_count_ == SIZE) return nullptr;

    for(std::size_t i=0; i<SIZE; ++i){
        if (free_map_[i]){
            free_map_[i] = false;
            used_count_++;
            return &pool_[i];//return the physical memory address

        }
    }
    return nullptr;
}

void deallocate(T* ptr){
    // Safety check: Ensure the pointer actually belongs to our pool
    if(ptr >= &pool_[0] && ptr <= &pool_[SIZE-1]){
        // Pointer arithmetic to find the exact array index
        std::size_t index =ptr - &pool_[0];
        if(!free_map_[index]){
            free_map_[index] = true;
            used_count_--;
        }
    }
}
std:: size_t used() const {return used_count_;}
};


template <typename T, std::size_t SIZE>
class CircularBuffer{
private:
    T buffer_[SIZE];
    std::size_t read_idx_ =0;
    std::size_t write_idx_ =0;
    std::size_t count_ =0;

public:
    bool push(const T& item){
        if (is_full()) return false;
        buffer_[write_idx_] = item;
        write_idx_ = (write_idx_+1) % SIZE;
        count_++;
        return true;
    }

    bool pop(T& out){
        if(is_empty()) return false;
        out = buffer_[read_idx_];
        read_idx_ = (read_idx_ +1) % SIZE;
        count_--;
        return true;
    }
    bool is_full() const {return count_ == SIZE;}
    bool is_empty() const {return count_ == 0;}
    std::size_t size() const {return count_;}
};

class Ros2ControlNode{
private:
    PoolAllocator<CANMessage, 16>pool_; //manage the actual memory
    CircularBuffer<CANMessage*, 16> queue_; // passes the pointers

public:
    void publish(uint32_t id, const char* data_str, uint8_t length, double timestamp){
        //1. claim a physical memeory slot from the pool
        CANMessage* msg_ptr = pool_.allocate();
        if(msg_ptr == nullptr){
            std::cout << "[ERROR]Memory pool exhausted" <<std::endl;
            return;
        }
        // 2. Fill the data into the claimed slot
        msg_ptr->id=id;
        msg_ptr->length =length;
        msg_ptr->timestamp = timestamp;
        std::memcpy(msg_ptr->data, data_str, length);

        if (!queue_.push(msg_ptr)) {
            std::cout << "[ERROR] Queue Full! Dropping message." << std::endl;
            pool_.deallocate(msg_ptr); // Clean up if queue is full
        }
    }

    void process(){
        CANMessage* msg_ptr = nullptr;

        //1.pop the pointer from the queue
        if(queue_.pop(msg_ptr)){
            //2. process/print the msg
            msg_ptr->print();
            //3. Given the memory block to the pool for reuse
            pool_.deallocate(msg_ptr);
        }
    }
    void status() const {
        std::cout << "--- [SYSTEM STATUS] Pool Used: " << pool_.used() 
                  << "/16 | Buffer Queue: " << queue_.size() << "/16 ---" << std::endl;
    }
};

int main() {
    Ros2ControlNode node;

    std::cout << "\n===== 1. PUBLISHING 6 MESSAGES =====" << std::endl;
    node.publish(0x101, "IMU1", 4, 1.001);
    node.publish(0x102, "IMU2", 4, 1.002);
    node.publish(0x201, "ENC1", 4, 1.003);
    node.publish(0x202, "ENC2", 4, 1.004);
    node.publish(0x301, "CAM1", 4, 1.005);
    node.publish(0x302, "CAM2", 4, 1.006);
    
    node.status(); // Should be: Pool 6, Buffer 6

    std::cout << "\n===== 2. PROCESSING QUEUE =====" << std::endl;
    for (int i = 0; i < 6; ++i) {
        node.process();
    }
    
    node.status(); // Should be: Pool 0, Buffer 0

    std::cout << "\n===== 3. PROVING MEMORY REUSE =====" << std::endl;
    node.publish(0x401, "LDR1", 4, 2.001);
    node.publish(0x402, "LDR2", 4, 2.002);
    node.publish(0x403, "LDR3", 4, 2.003);
    
    node.status(); // Should be: Pool 3, Buffer 3

    return 0;
}