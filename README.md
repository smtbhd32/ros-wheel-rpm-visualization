# Differential Drive RPM Visualization

This project includes three scripts designed to subscribe, generate, exchange, and plot the left and right target RPMs of a differential drive robot. The project consists of two C++ scripts (Script A and Script B) and one Python script (Script C). These scripts interact with each other using shared memory for communication between the C++ scripts and HTTP requests between the C++ and Python scripts. The objective is to calculate and visualize the robot's wheel RPMs in real time, without relying on pub-sub architectures like ROS, DDS, or MQTT.

<div align="center">
  <a href="https://youtu.be/Zvpnq7ACSOs">
    <img src="https://img.youtube.com/vi/Zvpnq7ACSOs/0.jpg" alt="Watch the video">
  </a>
</div>

## File Structure

```
assignment/
├── src/
│   ├── script_a/
│   │   ├── script_a.cpp         # C++ Script A - Receives cmd_vel and calculates wheel velocities
│   ├── script_b/
│   │   ├── script_b.cpp         # C++ Script B - Fetches shared data and sends it via HTTP request
│   ├── script_c/
│   │   ├── script_c.py          # Python Script C - Fetches data via HTTP request and plots wheel RPMs
├── include/
│   ├── shared_data.hpp          # Shared memory structure for interprocess communication
│   ├── httplib.hpp              # Header file for the HTTP client library to send and receive HTTP requests in C++ scripts
├── CMakeLists.txt               # CMake build configuration for C++ scripts
└── package.xml                  # ROS package metadata (if needed for your setup)
```

---

## System Description

### Script A (C++)

- **Functionality**: Subscribes to the `cmd_vel` topic and calculates the target left and right wheel RPMs based on received linear and angular velocities. Shares the data with other C++ scripts via shared memory.
- **Key Calculations**: Converts the target linear and angular velocities into wheel RPMs based on the given wheel-to-wheel distance and wheel diameter.
- **Data Shared**: Target left and right RPMs, input target linear velocity, angular velocity, and a timestamp.

### Script B (C++)

- **Functionality**: Fetches the shared data from Script A, prints the data, and sends it to Script C via HTTP requests.
- **Communication**: HTTP GET requests are sent from Script B to Script C to transmit the data.
- **Features**: Runs in a loop at 10 Hz, fetching data from shared memory, printing it to the console, and making HTTP requests to Script C.

### Script C (Python)

- **Functionality**: Hosts an HTTP server that listens for requests from Script B. Fetches the wheel RPM data from Script B and generates real-time plots of the left and right wheel RPM values.
- **Features**: Uses `requests` to make HTTP requests to Script B and `matplotlib` for plotting the RPM data.
- **API Request**: Fetches data from Script B through a `/get_data_from_B` endpoint and processes it for visualization.

---

## Data Exchange Mechanism

- **Shared Memory**: Script A shares the calculated wheel RPMs and other relevant data with Script B using shared memory.
- **HTTP Requests**: Script B sends the data to Script C using HTTP GET requests. Script C fetches the data via its `/get_data_from_B` API endpoint.

---

## Communication Protocol

- **Request Type**: HTTP GET
- **Request URL**: `/get_data_from_B`
- **Request Format**: JSON
  - Left wheel RPM
  - Right wheel RPM
  - Input linear velocity (m/s)
  - Input angular velocity (rad/s)
  - Timestamps generated by Script A and Script B

---

## How to Run

### Step 1: Build C++ Scripts

To compile the C++ scripts, use the following command from the root directory of the project:

```bash
colcon build --packages-select your_package_name
```

### Step 2: Source the Workspace

Once the build is complete, source the ROS workspace:

```bash
source install/setup.bash
```

### Step 3: Running the Scripts

- **Run Script A**: This script will subscribe to `cmd_vel` messages, perform the necessary calculations, and write data to shared memory.
  ```bash
  ros2 run your_package_name script_a
  ```

- **Run Script B**: This script will fetch data from Script A and send it to Script C via HTTP requests.
  ```bash
  ros2 run your_package_name script_b
  ```

- **Run Script C**: The Python script fetches the data from Script B and generates plots. 

**For Script C (Python):**

1. First, navigate to the `script_c` directory:
   ```bash
   cd assignment/src/script_c/
   ```

2. Then, run the Python script:
   ```bash
   python3 script_c.py
   ```

   **Note**: If the script doesn't run, make sure it's executable by using the following command:
   ```bash
   chmod +x script_c.py
   ```

3. Once the script is executed, it will fetch the wheel RPM data and plot it in real time.

---

## Performance and Reliability

- **Graceful Restart**: The system is designed to handle script restarts without losing data or affecting performance. If any script (A, B, or C) is killed, it can be restarted, and the data exchange will continue without disruption.
- **Error Handling**: The system ensures that if an error occurs during data fetching or communication, the process continues running, and errors are logged appropriately.

---
