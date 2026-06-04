## Day 1 — Memory & Real-Time Systems
* **Concepts:** `unique_ptr`, `shared_ptr`, `weak_ptr`, pool allocator
* **Built:** RealTimeSensorPipeline with watchdog fault detection
* **Key insight:** Heap allocation is often banned in real-time control loops; pre-allocated memory pools provide O(1) constant-time memory safety.