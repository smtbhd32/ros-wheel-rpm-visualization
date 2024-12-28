# Script C - README

## Overview

`script_c.py` is a Python script designed to fetch data from an API and plot the received left and right RPM values over time. It retrieves data from an external server and displays the RPM values in real-time on three separate subplots. This script handles network errors, maintains a history of the last known RPM values, and logs the fetched data for future analysis. 

## Features

1. **Fetch Data**: The script fetches the left and right wheel RPM values, along with additional data like linear and angular velocity, from a remote API.
2. **Real-Time Plotting**: It continuously updates and plots the left RPM, right RPM, and combined RPM values on three subplots in a single window.
3. **Error Handling**: The script handles network issues gracefully by using the last known values if the connection to the server fails.
4. **Data Logging**: The fetched data is logged for further analysis.

## Requirements

Before running the script, install the required dependencies using the `requirements.txt` file in the same folder:

```bash
pip install -r requirements.txt
```

Dependencies:
- `requests` – To make HTTP requests and fetch data from the API.
- `matplotlib` – For plotting the data in real-time.
- `time` – For time-based operations.
- `logging` – For logging errors and data fetch events.

## Installation

1. Clone or download the repository.
2. Install the necessary dependencies by running:
    ```bash
    pip install -r requirements.txt
    ```

3. Run the script:
    ```bash
    python script_c.py
    ```

## Script Execution

- The script will attempt to connect to the API hosted at `http://localhost:8080/get_data_from_B` to fetch data continuously.
- The data includes left and right RPM values, which will be plotted on the graph in real-time.
- If the connection to the API fails, the script will use the last known RPM values.
- The RPM values are plotted in blue (left RPM) and red (right RPM) on the respective subplots.
- The script displays the graph with the following axes:
  - **Left RPM**: Displays the left wheel RPM over time.
  - **Right RPM**: Displays the right wheel RPM over time.
  - **Combined RPM**: Displays both left and right RPM values on a single plot.

## Logging

- Logs of fetched data and errors are stored in a file named `data_fetch.log`.
- In case of network issues, error messages will be logged.

## API Interaction

The script interacts with the following API endpoint:

```http
GET http://localhost:8080/get_data_from_B
```

This endpoint returns a JSON response with the following data:

- `linear_velocity`: Linear velocity of the robot.
- `angular_velocity`: Angular velocity of the robot.
- `left_rpm`: RPM of the left wheel.
- `right_rpm`: RPM of the right wheel.
- `timestamp_a`: Timestamp of the robot's data.
- `timestamp_b`: Timestamp of the API data.

The script fetches data at regular intervals and updates the plot with the new values.

## Error Handling

- If the script fails to fetch data from the API, it logs the error and uses the last known RPM values.
- If the script fails repeatedly (more than 50 attempts), a message stating "No connection" is displayed, and the script continues using the last known values.

## Interrupting the Script

To stop the script, press `Ctrl+C`. The script will log the interruption and close the plot window gracefully.

## Functions

### `fetch_data(url)`
This function fetches data from the provided API endpoint.

- **Parameters**:
  - `url`: The URL to fetch data from.
  
- **Returns**:
  - A dictionary containing the fetched data (`left_rpm`, `right_rpm`, `linear_velocity`, `angular_velocity`, `timestamp_a`, `timestamp_b`), or `None` if there was an error during the request.

- **Description**:
  - This function sends a `GET` request to the specified URL and processes the returned JSON response. It checks for the required keys (`left_rpm`, `right_rpm`, `timestamp_b`) in the response. If the request fails or if the expected data is not found, it logs the error and returns `None`.

### `update_plot(frame)`
This function is called by `FuncAnimation` at regular intervals to update the plot with the latest data.

- **Parameters**:
  - `frame`: The current frame number (used for animation, but not utilized in this function).
  
- **Returns**:
  - `left_rpm_plot, right_rpm_plot`: The updated plot objects for the left and right RPM.

- **Description**:
  - This function fetches the latest data using the `fetch_data` function. If data is successfully retrieved, it updates the `left_rpm` and `right_rpm` values, appends them to their respective lists (`left_rpm_data` and `right_rpm_data`), and updates the plots. If the API fails to provide data, the function uses the last known values and continues plotting. It also manages the number of data points displayed on the plot, limiting it to the most recent 50 points.

### `FuncAnimation`
- **Parameters**:
  - `fig`: The figure object to animate.
  - `update_plot`: The function to update the plot at regular intervals.
  - `interval`: The interval in milliseconds between updates (set to 100ms in this case).
  - `cache_frame_data`: Set to `False` to avoid caching of frames.

- **Description**:
  - `FuncAnimation` from `matplotlib.animation` is used to update the plot in real-time. It continuously calls the `update_plot` function at the specified interval and redraws the plot with new data.

--- 
