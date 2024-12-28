# Script A - README

## Overview

This script calculates the target velocities for a robot's left and right wheels based on the received linear and angular velocities. It also shares the calculated wheel velocities, input velocities, and timestamp data through shared memory, allowing other processes to access this information. The script subscribes to the `cmd_vel` topic, listens for velocity commands, and publishes the corresponding wheel RPM values to shared memory.

### Key Features:
- **Wheel Base & Diameter**: Customizable distance between wheels and wheel diameter for velocity calculations.
- **Shared Memory**: Communicates computed values (wheel RPM, linear velocity, angular velocity, timestamp) via shared memory for interprocess communication.
- **Graceful Shutdown**: Supports graceful shutdown through signal handling for SIGINT (Ctrl+C).

---

## Prerequisites

Before running the script, ensure that you have the following dependencies installed:

1. **ROS 2**: Tested on [ROS 2 version](https://index.ros.org/doc/ros2/Installation/).
2. **Boost Libraries**: Used for interprocess communication (`boost::interprocess`).
3. **C++ Standard Libraries**: For memory management and thread handling.

---

## File Structure

```
assignment/
├── include/
│   ├── shared_data.hpp    # Shared memory structure for interprocess communication
├── src/
│   ├── script_a/
│   │   ├── script_a.cpp    # ROS 2 node for receiving velocity commands and publishing data
├── CMakeLists.txt         # ROS 2 CMake build file
└── package.xml            # ROS 2 package metadata
```

---

## Dependencies

### ROS 2 and Message Includes

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
```

### Standard Includes

```cpp
#include <chrono>
#include <cmath>
#include <mutex>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <thread>  // For std::this_thread
#include <csignal>  // For signal handling (e.g., SIGINT)
```

### Boost Libraries for IPC (Interprocess Communication)

```cpp
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
```

---

## Core Functions

### 1. `signalHandler(int signal)`

Handles termination signals (such as SIGINT from Ctrl+C). Ensures that shared memory is cleaned up before shutting down the application.

- **Parameters**: 
  - `signal`: The signal that triggered the handler.

- **Purpose**: 
  - Safely shuts down ROS 2 and sets the `shutdown_requested` flag.

---

### 2. `SharedData* initializeSharedMemory()`

Initializes or opens the shared memory region for interprocess communication. If shared memory doesn't exist, it will create a new one.

- **Returns**: 
  - A pointer to the `SharedData` structure that will hold the shared data.

- **Purpose**: 
  - Sets up a shared memory region to hold velocity data (left and right wheel RPM, linear/ angular velocities, and timestamp).

---

### 3. `class VelocityCalculator : public rclcpp::Node`

The main ROS 2 node that listens to velocity commands and computes wheel velocities based on the input. It updates the shared memory with the calculated wheel RPM and other related data.

#### Constructor: 

```cpp
VelocityCalculator(SharedData* shared_data)
```
- **Parameters**: 
  - `shared_data`: Pointer to the shared memory structure for storing computed data.

- **Purpose**: 
  - Subscribes to the `cmd_vel` topic to listen for velocity commands.

#### Method: `cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)`

This method is called when a new message is received on the `cmd_vel` topic. It computes the left and right wheel velocities in RPM and updates the shared memory.

- **Parameters**: 
  - `msg`: The `geometry_msgs::msg::Twist` message that contains the target linear and angular velocities.

- **Purpose**: 
  - Computes wheel velocities (left and right) in RPM based on linear and angular velocities.
  - Updates the shared memory with the computed values and timestamp.

---

## Shared Memory Structure (`shared_data.hpp`)

The `shared_data.hpp` file defines a custom structure for holding the shared data between processes. It includes the following fields:

```cpp
struct SharedData {
    double left_wheel_rpm;        // RPM of the left wheel
    double right_wheel_rpm;       // RPM of the right wheel
    double linear_velocity;       // Linear velocity (m/s)
    double angular_velocity;      // Angular velocity (rad/s)
    long timestamp;               // Epoch timestamp (milliseconds)
    std::mutex mtx;               // Mutex to protect shared data
};
```

- **Purpose**: 
  - This structure is used to store the computed wheel velocities, input velocities, and timestamp, which can be accessed by other programs via shared memory.

---

## How to Run

1. **Build the ROS 2 package**:
   - Navigate to the root of your ROS 2 workspace and build the package:
   
     ```bash
     colcon build --packages-select assignment
     ```

2. **Source the workspace**:
   - Before running the script, source your ROS 2 workspace:

     ```bash
     source install/setup.bash
     ```

3. **Run the node**:
   - Use the following command to run the `script_a_node`:

     ```bash
     ros2 run assignment script_a_node
     ```

---
