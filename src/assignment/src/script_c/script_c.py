import requests
import time
import logging
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# API endpoint
url = "http://localhost:8080/get_data_from_B"

# Initialize plot with 3 subplots
fig, (ax, bx, cx) = plt.subplots(3, 1, sharex=True)

# Initialize plots for Left RPM and Right RPM
left_rpm_plot, = ax.plot([], [], label="Left RPM", color='b')
right_rpm_plot, = bx.plot([], [], label="Right RPM", color='r')

# Labels and Legends
ax.set_ylabel("Left RPM")
bx.set_ylabel("Right RPM")
cx.set_ylabel("Combined RPM")
cx.set_xlabel("Time")
ax.legend()
bx.legend()

# Set Y-axis limits
ax.set_ylim(-10, 60)
bx.set_ylim(-10, 60)
cx.set_ylim(-10, 60)

# Data storage
left_rpm_data = []
right_rpm_data = []
timestamps = []

# Last known RPM values in case of network failure
last_left_rpm = None
last_right_rpm = None
retries = 0

# Setup logging
logging.basicConfig(filename="data_fetch.log", level=logging.INFO,
                    format="%(asctime)s - %(levelname)s - %(message)s")

# Limit the number of data points displayed on the plot
MAX_DATA_POINTS = 50

# Connection status flag
error_printed = False

def fetch_data(url):
    """
    Fetches data from the API with error handling. No retry loop.
    """
    try:
        response = requests.get(url, timeout=0.01)
        response.raise_for_status()  # Raise HTTPError for bad status codes
        data = response.json()
        # Validate required keys in the JSON response
        if "left_rpm" not in data or "right_rpm" not in data or "timestamp_b" not in data:
            raise ValueError("Invalid data format: Missing keys in response.")
        return data
    except requests.exceptions.RequestException as e:
        # Log the error and return None if there's a network issue
        logging.error(f"Network error: {e}")
        return None
    except ValueError as ve:
        # Log data format issues 
        logging.error(f"Data error: {ve}")
        return None

def update_plot(frame):
    """
    Update the plot with new data fetched from the API.
    If no data is fetched, continue with the last known values or skip the plot for that interval.
    """
    global last_left_rpm, last_right_rpm, error_printed, retries  # Use global last known RPM values and connection status

    data = fetch_data(url)
    
    if data:
        retries = 0
        if error_printed == True:
            print("Connected to server")
            error_printed = False

        # Extract values from JSON
        linear_velocity = data["linear_velocity"]
        angular_velocity = data["angular_velocity"]
        left_rpm = data["left_rpm"]
        right_rpm = data["right_rpm"]
        timestamp_a = data["timestamp_a"]
        timestamp_b = data["timestamp_b"]

        # Print formatted output
        print(f"Linear Velocity: {linear_velocity:.4f} m/s, Angular Velocity: {angular_velocity:.4f} rad/s, "
            f"Left RPM: {left_rpm:.4f}, Right RPM: {right_rpm:.4f}, "
            f"TimestampA: {timestamp_a}, TimestampB: {timestamp_b}")
        
        logging.info(f"Left RPM: {left_rpm}, Right RPM: {right_rpm}, Timestamp: {timestamp_a}")

        # Update last known RPM values
        last_left_rpm = left_rpm
        last_right_rpm = right_rpm

        # Append new data
        left_rpm_data.append(left_rpm)
        right_rpm_data.append(right_rpm)
        timestamps.append(time.time())
    else:
        retries = retries + 1
        if retries > 50:
            # If no data fetched (due to network issues), set None for both RPMs
            if error_printed == False:
                print("No connection")
                error_printed = True
            left_rpm_data.append(None)  
            right_rpm_data.append(None)  
            timestamps.append(time.time())
        else:
            # Append new data
            left_rpm_data.append(last_left_rpm)
            right_rpm_data.append(last_right_rpm)
            timestamps.append(time.time())

    # If the number of data points exceeds the max limit, remove the oldest one
    if len(left_rpm_data) > MAX_DATA_POINTS:
        left_rpm_data.pop(0)
        right_rpm_data.pop(0)
        timestamps.pop(0)

    # Set new data for the plot
    left_rpm_plot.set_xdata(timestamps)
    left_rpm_plot.set_ydata(left_rpm_data)
    right_rpm_plot.set_xdata(timestamps)
    right_rpm_plot.set_ydata(right_rpm_data)

    # Update the X-axis limits to show a fixed time window (e.g., last 20 seconds)
    current_time = time.time()
    TIME_WINDOW = 10
    ax.set_xlim(current_time - TIME_WINDOW, current_time)
    bx.set_xlim(current_time - TIME_WINDOW, current_time)
    cx.set_xlim(current_time - TIME_WINDOW, current_time)


    # Set new data for the plot a
    ax.plot(timestamps, left_rpm_data, label="Left RPM", color='b')

    # Set new data for the plot b
    bx.plot(timestamps, right_rpm_data, label="Right RPM", color='r')
    
    # Set new data for the plot c
    cx.plot(timestamps, left_rpm_data, label="Left RPM", color='b')
    cx.plot(timestamps, right_rpm_data, label="Right RPM", color='r')

    return left_rpm_plot, right_rpm_plot

# Set up the animation
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)  # Update every 100ms (0.1 sec)

# Add legend to the plots
ax.legend(loc='upper right')

# Ensure the plot is visible even before data is fetched
plt.draw()

# Handle keyboard interrupt gracefully
try:
    plt.show()
except KeyboardInterrupt:
    print("\nCtrl + C pressed. Shutting down gracefully.\n")
    logging.info("Process interrupted by user.")
    plt.close(fig)
    exit(0)
