#include <iostream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <thread>
#include <chrono>
#include <httplib.h>
#include <csignal>
#include <atomic>
#include "shared_data.hpp"

namespace ipc = boost::interprocess;

// Global atomic flag for termination, shared across threads
std::atomic<bool> terminate_flag(false);

// Signal handler to handle Ctrl+C and set termination flag
void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl + C pressed. Shutting down gracefully...\n";
        terminate_flag.store(true);
    }
}

// Main class for fetching velocity data and managing server operations
class VelocityFetcher {
public:
    // Main application loop
    void run() {
        // Start HTTP server in a separate thread
        std::thread server_thread(&VelocityFetcher::start_http_server, this);

        // Main loop to check shared memory and fetch data
        while (!terminate_flag.load()) {
            check_new_data(); // Check for new data
            if (new_data_available_) {
                print_data(); // Print fetched data
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Maintain a 10 Hz loop
        }

        // Stop the HTTP server gracefully
        if (server_started_) {
            svr_->stop();
            server_thread.join();
        }
        std::cout << "Application terminated.\n";
    }

private:
    // Check if shared memory is available
    void check_shared_memory() {
        try {
            // Attempt to open shared memory
            shm_ = ipc::shared_memory_object(ipc::open_only, "SharedMemory", ipc::read_write);
            region_ = ipc::mapped_region(shm_, ipc::read_write);
            shared_data_ = static_cast<SharedData*>(region_.get_address());
            if (!shared_memory_available_) {
                shared_memory_available_ = true;
                message_printed_ = false;
                std::cout << "Shared memory is now available.\n";
            }
        } catch (const ipc::interprocess_exception&) {
            // Handle cases where shared memory is unavailable
            if (shared_memory_available_) {
                shared_memory_available_ = false;
                std::cout << "Shared memory not available.\n\n";
                message_printed_ = true;
            } else if (!message_printed_) {
                std::cout << "Shared memory not available.\n\n";
                message_printed_ = true;
            }
        }
    }

    // Check for new data by comparing timestamps
    void check_new_data() {
        check_shared_memory();
        if (shared_memory_available_) {
            load_data(); // Load data from shared memory
            if (timestamp_a != last_timestamp_a_) {
                if (!new_data_available_ && retries > 10) {
                    retries = 0;
                    std::cout << "New data available. Continuing...\n";
                }
                last_timestamp_a_ = timestamp_a;  // Update the last timestamp
                message_printed_ = false;
                new_data_available_ = true;
            } else {
                new_data_available_ = false;
                retries++;
                if (!message_printed_ && retries > 10) {
                    std::cout << "New data not available.\n\n";
                    message_printed_ = true;
                }
            }
        }
    }

    // Print the fetched data to the console
    void print_data() {
        printf("Linear Velocity: %.4f m/s, Angular Velocity: %.4f rad/s, Left RPM: %.4f, Right RPM: %.4f, TimestampA: %ld, TimestampB: %ld\n",
               linear_velocity, angular_velocity, left_rpm, right_rpm, timestamp_a, timestamp_b);
    }

    // Load data from the shared memory structure
    void load_data() {
        std::lock_guard<std::mutex> lock(shared_data_->mtx); // Thread-safe access
        left_rpm = shared_data_->left_wheel_rpm;
        right_rpm = shared_data_->right_wheel_rpm;
        linear_velocity = shared_data_->linear_velocity;
        angular_velocity = shared_data_->angular_velocity;
        timestamp_a = shared_data_->timestamp;
        auto now = std::chrono::system_clock::now();
        timestamp_b = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    }

    // Start the HTTP server to provide velocity data
    void start_http_server() {
        httplib::Server svr;
        svr_ = &svr;

        // Define the HTTP endpoint
        svr.Get("/get_data_from_B", [&](const httplib::Request& req, httplib::Response& res) {
            try {
                if (new_data_available_) {
                    // Return JSON response with velocity data
                    std::string response = "{\"linear_velocity\": " + std::to_string(linear_velocity) +
                                            ", \"angular_velocity\": " + std::to_string(angular_velocity) +
                                            ", \"left_rpm\": " + std::to_string(left_rpm) +
                                            ", \"right_rpm\": " + std::to_string(right_rpm) +
                                            ", \"timestamp_a\": " + std::to_string(timestamp_a) +
                                            ", \"timestamp_b\": " + std::to_string(timestamp_b) + "}";
                    res.set_content(response, "application/json");
                } else {
                    // Handle no new data scenario
                    res.status = 500;
                    res.set_content("{\"error\": \"no new data\"}", "application/json");
                }
            } catch (const std::exception& e) {
                // Handle exceptions gracefully
                res.status = 500;
                res.set_content("{\"error\": \"" + std::string(e.what()) + "\"}", "application/json");
            }
        });

        // Start listening on port 8080
        try {
            std::cout << "Starting HTTP server on port 8080...\n";
            server_started_ = svr.listen("0.0.0.0", 8080);
            if (!server_started_) {
                throw std::runtime_error("Failed to start the HTTP server.");
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }

    // Shared memory variables
    ipc::shared_memory_object shm_;
    ipc::mapped_region region_;
    SharedData* shared_data_;

    // HTTP server pointer and state variables
    httplib::Server* svr_;
    bool shared_memory_available_ = false;   // Flag to track shared memory availability
    bool message_printed_ = false;           // Flag to track if the message has already been printed
    bool server_started_ = false; 

    // Data variables
    double left_rpm = 0;
    double right_rpm = 0;
    double linear_velocity = 0;
    double angular_velocity = 0;
    uint64_t timestamp_a = 0;
    uint64_t timestamp_b = 0;
    uint64_t last_timestamp_a_ = 0;
    bool new_data_available_ = false;
    int retries = 0;
};

int main() {
    // Register signal handler
    std::signal(SIGINT, signal_handler);

    // Create and run the VelocityFetcher
    VelocityFetcher velocity_fetcher;
    velocity_fetcher.run();
    return 0;
}
