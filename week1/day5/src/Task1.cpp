#include<iostream>
#include<vector>
#include<utility> //For std::move

class SensorBuffer{
private:
    std::vector<double>payload_;

public:
    //standard constructor
    SensorBuffer(size_t size){
        payload_ = std::vector<double>(size, 0.0);
        std::cout << "[INIT] Created buffer with "<< payload_.size() << " elements."<<std::endl;

    }
    
    // copy constructor old way 
    SensorBuffer(const SensorBuffer& other){
        // This physically duplicates all 1000 elements in memory
        payload_ = other.payload_;
        std::cout << "[COPY] COPY — expensive! Duplicated " << payload_.size() << " elements." << std::endl;
    }

    //3. Move constructor 
    // "noexcept" is a promise to the compiler that stealing this memory will never throw an error

    SensorBuffer(SensorBuffer&& other) noexcept{

        payload_= std::move(other.payload_);
        std::cout << "[MOVE] MOVE — free! Stole pointer to " << payload_.size() << " elements." << std::endl;

    }
    // Helper method to check the current size
    size_t size() const{
        return payload_.size();
    }
};


int main(){
    std::cout << "===== C++ MOVE SEMANTICS DEMO =====" << std::endl;

    // 1. Create the initial buffer
    std::cout << "\n1. Creating buffer_a..." << std::endl;
    SensorBuffer buffer_a(1000);
    std::cout << "   buffer_a size: " << buffer_a.size() << std::endl;

    //2. Trigger the copy constructor 
    std::cout << "\n2. Copying buffer_a into buffer_b..." << std::endl;
    SensorBuffer buffer_b = buffer_a; // Calls Copy Constructor

    std::cout << "   buffer_a size: " << buffer_a.size() << " (Still intact)" << std::endl;
    std::cout << "   buffer_b size: " << buffer_b.size() << std::endl;

    // 3. Trigger the Move Constructor
    std::cout << "\n3. Moving buffer_a into buffer_c..." << std::endl;
    // std::move() casts buffer_a into a temporary rvalue, forcing the Move Constructor
    SensorBuffer buffer_c = std::move(buffer_a); 

    std::cout << "   buffer_c size: " << buffer_c.size() << " (Successfully inherited memory)" << std::endl;

    // 4. Prove the original is empty
    std::cout << "\n4. Checking original buffer_a..." << std::endl;
    std::cout << "   buffer_a size: " << buffer_a.size() << " (Hollowed out!)" << std::endl;

    std::cout << "\n===== DEMO COMPLETE =====" << std::endl;

    return 0;
}