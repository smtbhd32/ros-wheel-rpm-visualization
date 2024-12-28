#ifndef SHARED_DATA_HPP
#define SHARED_DATA_HPP

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <mutex>

namespace ipc = boost::interprocess;

// Define the shared data structure
struct SharedData {
    double left_wheel_rpm;
    double right_wheel_rpm;
    double linear_velocity;
    double angular_velocity;
    uint64_t timestamp;

    // Mutex for thread-safe access
    std::mutex mtx;

    // Constructor for initializing shared memory
    SharedData()
        : left_wheel_rpm(0), right_wheel_rpm(0), 
          linear_velocity(0), angular_velocity(0), 
          timestamp(0) {}
};

#endif // SHARED_DATA_HPP
