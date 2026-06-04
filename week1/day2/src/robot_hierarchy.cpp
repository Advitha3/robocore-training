#include <iostream>


template<typename Derived>
class CRTPSensor {
public:
    // The Interface
    double read() const {
        // The Magic: We statically cast 'this' pointer to the Derived type.
        // Because this is a template, the compiler knows exactly what Derived is at compile time.
        return static_cast<const Derived*>(this)->read_impl();
    }

    const char* name() const {
        return static_cast<const Derived*>(this)->name_impl();
    }

    // Shared functionality lives once in the Base class, avoiding code duplication!
    void log() const {
        std::cout << "[CRTP Telemetry] " << name() << ": " << read() << std::endl;
    }
};

// 2. The Derived Class
// Notice the inheritance: It inherits from CRTPSensor, and passes ITSELF as the template argument.
class CRTPImu : public CRTPSensor<CRTPImu> {
public:
    // We implement the specific functions the base class expects.
    // No 'virtual' keyword, no 'override' keyword!
    double read_impl() const { 
        return 9.81; 
    }
    
    const char* name_impl() const { 
        return "CRTPImu"; 
    }
};

class CRTPLidar : public CRTPSensor<CRTPLidar> {
public:
    double read_impl() const { 
        return 3.45; 
    }
    
    const char* name_impl() const { 
        return "CRTPLidar"; 
    }
};

// ==========================================
// MAIN FUNCTION
// ==========================================
int main() {
    std::cout << "===== STARTING CRTP ARCHITECTURE =====" << std::endl;

    // 1. Created directly on the stack (No 'new', zero heap allocation)
    CRTPImu   imu;
    CRTPLidar lidar;

    // 2. Call the shared log() function from the Base class
    // No vtable lookup. Zero runtime overhead.
    imu.log();
    lidar.log();

    std::cout << "===== CRTP ARCHITECTURE ENDING =====" << std::endl;
    return 0;
}