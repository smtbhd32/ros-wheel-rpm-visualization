# Script B - Velocity Fetcher

## Overview

Script B is a C++ application that fetches velocity data from a shared memory segment, prints it, and exposes the data through an HTTP server. It fetches the data continuously at 10 Hz and allows external clients to access it through a simple REST API. The data includes linear velocity, angular velocity, and motor RPMs, which are received from Script A.

## Features

- Fetches velocity data (linear and angular velocities, left and right RPMs, timestamps) from shared memory.
- Exposes the fetched data through a REST API over HTTP.
- Operates continuously in a loop with a frequency of 10 Hz.
- Allows external access to the fetched data via the `/get_data_from_B` endpoint.
- Handles graceful shutdown upon receiving `Ctrl + C`.

## Requirements

- **ROS 2** (Foxy or later)
- **Boost C++ Libraries**
- **httplib** (HTTP server functionality, located in `assignment/include/httplib.hpp`)
  
Make sure these dependencies are available before running the script.

## File Structure

```
assignment/
├── src/
│   └── script_b/
│       └── script_b.cpp
└── include/
│   └── httplib.hpp           # Dependency for HTTP server functionality, required for Script B
├── CMakeLists.txt
```

## Installation

1. Clone the repository if you haven't already.
2. Build the package inside your ROS 2 workspace:
   ```bash
   colcon build --packages-select assignment
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Script

To run Script B, use the following command:

```bash
ros2 run assignment script_b_node
```

This command starts the script that fetches velocity data from shared memory, prints it to the console, and starts an HTTP server that listens on port `8080`.

### HTTP Endpoint

The following endpoint is available for external clients:

- **GET /get_data_from_B**
  - Returns the fetched velocity data in JSON format.
  - Sample response:
    ```json
    {
      "linear_velocity": 0.1234,
      "angular_velocity": 0.5678,
      "left_rpm": 120.0,
      "right_rpm": 118.5,
      "timestamp_a": 1616161616161,
      "timestamp_b": 1616161616165
    }
    ```

## Functions in Script B

### `main()`
- The entry point of the application.
- Calls the `VelocityFetcher` class to run the main logic of fetching data and serving it via HTTP.

### `VelocityFetcher::run()`
- The main loop that runs at 10 Hz.
- It continuously fetches the velocity data from shared memory and starts an HTTP server in a separate thread.
- If a `SIGINT` (Ctrl+C) signal is received, the server is gracefully shut down and the program exits.

### `VelocityFetcher::check_shared_memory()`
- This function checks for the existence of the shared memory region.
- If found, it initializes the connection to shared memory and prepares to fetch data from it.

### `VelocityFetcher::check_new_data()`
- Polls the shared memory to check if new data is available.
- This function reads the timestamp and compares it to the previous one to detect new data.

### `VelocityFetcher::load_data()`
- Fetches the velocity data (linear velocity, angular velocity, left RPM, and right RPM) from the shared memory segment.
- Updates the current data and prints it to the console.

### `VelocityFetcher::start_http_server()`
- Initializes and starts an HTTP server on port `8080`.
- Exposes the endpoint `/get_data_from_B` that serves the latest velocity data as a JSON response.

### `signal_handler(int signum)`
- A signal handler that allows graceful shutdown when `Ctrl + C` is pressed.
- Ensures the HTTP server is properly stopped before the program exits.

## Shutdown

Press `Ctrl + C` to stop the script. This triggers the signal handler, which gracefully shuts down the HTTP server and terminates the program.

---