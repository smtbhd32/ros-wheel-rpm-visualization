// Required ROS 2 and message includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Standard includes
#include <chrono>
#include <cmath>
#include <mutex>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <thread>  // For std::this_thread
#include <csignal>  // For signal handling (e.g., SIGINT)

// Boost for interprocess communication
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// Custom shared data structure
#include "shared_data.hpp"

// Alias for Boost interprocess namespace
namespace ipc = boost::interprocess;

// Atomic flag to manage graceful shutdown
std::atomic<bool> shutdown_requested(false);

// Constants for wheel and robot dimensions
const double WHEEL_BASE = 0.443;       // Distance between wheels in meters
const double WHEEL_DIAMETER = 0.181;  // Diameter of the wheels in meters

// Signal handler for clean shutdown on SIGINT
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nSIGINT received. Shutting down gracefully..." << std::endl;
        shutdown_requested = true;
        rclcpp::shutdown();  // Signal ROS 2 shutdown
    }
}

// Function to initialize shared memory
SharedData* initializeSharedMemory() {
    static ipc::mapped_region region; // Static to keep memory mapped throughout
    try {
        // Attempt to open existing shared memory
        ipc::shared_memory_object shm(ipc::open_only, "SharedMemory", ipc::read_write);
        std::cout << "Shared memory exists. Opening..." << std::endl;

        // Map the shared memory
        region = ipc::mapped_region(shm, ipc::read_write);
        SharedData* shared_data = static_cast<SharedData*>(region.get_address());

        if (!shared_data) {
            throw std::runtime_error("Failed to map existing shared memory.");
        }

        return shared_data;
    } catch (const ipc::interprocess_exception &e) {
        // If shared memory does not exist, create it
        std::cout << "Shared memory does not exist. Creating a new one..." << std::endl;

        try {
            ipc::shared_memory_object shm(ipc::create_only, "SharedMemory", ipc::read_write);
            shm.truncate(sizeof(SharedData)); // Set shared memory size

            region = ipc::mapped_region(shm, ipc::read_write);
            SharedData* shared_data = static_cast<SharedData*>(region.get_address());

            if (!shared_data) {
                throw std::runtime_error("Failed to map new shared memory.");
            }

            // Zero-initialize shared data
            std::memset(shared_data, 0, sizeof(SharedData));
            return shared_data;
        } catch (const std::exception &e) {
            std::cerr << "Error during shared memory creation: " << e.what() << std::endl;
            return nullptr;
        }
    }
}

// ROS 2 Node for calculating and publishing wheel velocities
class VelocityCalculator : public rclcpp::Node {
public:
    VelocityCalculator(SharedData* shared_data) 
        : Node("velocity_calculator"), shared_data(shared_data) {
        // Subscription to velocity command topic
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&VelocityCalculator::cmdVelCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Velocity Calculator Node Initialized.");
    }

private:
    // Callback function for processing velocity commands
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_velocity = msg->linear.x;   // Linear velocity (m/s)
        double angular_velocity = msg->angular.z; // Angular velocity (rad/s)

        // Compute wheel velocities
        double left_wheel_velocity = linear_velocity - (angular_velocity * WHEEL_BASE / 2.0);
        double right_wheel_velocity = linear_velocity + (angular_velocity * WHEEL_BASE / 2.0);

        // Convert to RPM
        double left_wheel_rpm = (left_wheel_velocity / (M_PI * WHEEL_DIAMETER)) * 60.0;
        double right_wheel_rpm = (right_wheel_velocity / (M_PI * WHEEL_DIAMETER)) * 60.0;

        // Get timestamp in milliseconds
        auto now = std::chrono::system_clock::now();
        auto epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        // Update shared data with mutex lock
        {
            std::lock_guard<std::mutex> lock(shared_data->mtx);
            shared_data->left_wheel_rpm = left_wheel_rpm;
            shared_data->right_wheel_rpm = right_wheel_rpm;
            shared_data->linear_velocity = linear_velocity;
            shared_data->angular_velocity = angular_velocity;
            shared_data->timestamp = epoch;
        }

        // Log computed values
        RCLCPP_INFO(this->get_logger(),
            "Linear Velocity: %.4f m/s, Angular Velocity: %.4f rad/s, Left RPM: %.4f, Right RPM: %.4f, Timestamp: %ld",
            linear_velocity, angular_velocity, left_wheel_rpm, right_wheel_rpm, epoch);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    SharedData* shared_data;  // Pointer to shared memory data structure
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Register signal handler for clean shutdown
    std::signal(SIGINT, signalHandler);

    // Initialize shared memory
    SharedData* shared_data = nullptr;
    try {
        shared_data = initializeSharedMemory();
        if (!shared_data) {
            throw std::runtime_error("Failed to initialize shared memory.");
        }
        std::cout << "Shared memory initialized successfully." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Create ROS node
    auto node = std::make_shared<VelocityCalculator>(shared_data);

    // Multi-threaded executor for callback handling
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Run executor in a separate thread
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Monitor shutdown flag
    while (!shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Join executor thread
    if (executor_thread.joinable()) {
        executor_thread.join();
    }

    // Clean up shared memory
    std::cout << "Cleaning up shared memory..." << std::endl;
    try {
        ipc::shared_memory_object::remove("SharedMemory");
        std::cout << "Shared memory cleaned up successfully." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error during shared memory cleanup: " << e.what() << std::endl;
    }

    std::cout << "Application shutdown complete." << std::endl;
    return 0;
}
