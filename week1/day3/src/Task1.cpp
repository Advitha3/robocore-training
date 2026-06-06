#include<iostream>
#include<cstddef>

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
        std::cout <<"[BUFFER] FULL- dropped:" << item << std::endl; 
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
        std::cout <<"[Buffer] EMPTY - nothing to read" <<std::endl;
        return false;
    }

    out = buffer_[read_idx_];

    //modulo wrap applies to the read index as well 
    read_idx_ = (read_idx_ +1)% SIZE;
    count_--;
    return true ;
}


    bool is_full() const {
        return count_ == SIZE;
    }

    bool is_empty() const{
        return count_ ==0;
    }
    std::size_t size() const{
        return count_;
    }

    void status() const {
        std::cout << "[Buffer Status] Elements: " << count_ << "/" << SIZE 
                  << " | Read Index: " << read_idx_ 
                  << " | Write Index: " << write_idx_ 
                  << (is_full() ? " [FULL]" : "")
                  << (is_empty() ? " [EMPTY]" : "")
                  << std::endl;
    }

};

int main() {
    std::cout << "===== 1. FILLING BUFFER =====" << std::endl;
    CircularBuffer<int, 8> buf;
    
    // 1. Push 8 ints (0-7) -> all succeed
    for (int i = 0; i < 8; ++i) {
        buf.push(i);
        std::cout << "Pushed: " << i << std::endl;
    }
    
    // 2. status() -> should show FULL, write=0 (wrapped!)
    std::cout << "\nStatus after 8 pushes:" << std::endl;
    buf.status();

    // 3. Try push one more -> should fail, print drop message
    std::cout << "\n===== 2. OVERFLOW TEST =====" << std::endl;
    buf.push(99); 

    // 4. Pop 3 items, print them -> should get 0, 1, 2 in order (FIFO)
    std::cout << "\n===== 3. READING DATA =====" << std::endl;
    int popped_val;
    for (int i = 0; i < 3; ++i) {
        if (buf.pop(popped_val)) {
            std::cout << "Popped: " << popped_val << std::endl;
        }
    }

    // 5. status() -> Elements: 5, read_idx=3
    std::cout << "\nStatus after popping 3 items:" << std::endl;
    buf.status();

    // 6. Push 3 new items (10,11,12) -> should reuse freed slots
    std::cout << "\n===== 4. REUSING FREED SLOTS =====" << std::endl;
    buf.push(10); std::cout << "Pushed: 10" << std::endl;
    buf.push(11); std::cout << "Pushed: 11" << std::endl;
    buf.push(12); std::cout << "Pushed: 12" << std::endl;

    // 7. status() -> Elements: 8, FULL again, write_idx=3
    std::cout << "\nStatus after reusing slots:" << std::endl;
    buf.status();

    return 0;
}
