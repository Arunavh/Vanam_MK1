import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
from collections import deque

# ------------------
# Configuration
# ------------------
SERIAL_PORT = "COM13"    # Update this port name (e.g., "COM5" on Windows or "/dev/ttyUSB0" on Linux/Mac)
BAUD_RATE = 115200

# ------------------
# Global Data Buffers and Latest Data Storage
# ------------------
# Time series buffers for graphs (we'll record telemetry altitude, temperature, pressure)
window_size = 50
alt_ts = deque(maxlen=window_size)
temp_ts = deque(maxlen=window_size)
pres_ts = deque(maxlen=window_size)

# Latest full telemetry data (list of 20 floats)
latest_data = None

# Open log file (data will be appended)
log_file = open("telemetry_log.txt", "a")

# ------------------
# Serial Connection
# ------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ------------------
# Utility: Validate Incoming Data
# ------------------
def is_valid_data(line):
    # Check if the line contains exactly 20 values (19 commas)
    if line.count(',') != 19:
        return False
    try:
        # Verify that all fields can be converted to float
        list(map(float, line.split(',')))
        return True
    except ValueError:
        return False

# ------------------
# Function: Draw a Gauge (Dial)
# ------------------
def draw_gauge(ax, value, min_val, max_val, title):
    # Clear the axis and remove spines/labels
    ax.clear()
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-0.2, 1.2)
    ax.axis('off')
    
    # Draw the gauge arc (a semicircular arc from 0° to 180°)
    arc = patches.Arc((0,0), 2, 2, theta1=0, theta2=180, linewidth=2)
    ax.add_patch(arc)
    
    # Calculate the needle angle (in degrees) corresponding to the value
    # Map the value from [min_val, max_val] to an angle in [180, 0]
    angle = 180 * (1 - (value - min_val) / (max_val - min_val))
    angle_rad = np.deg2rad(angle)
    
    # Determine needle end point (length = 0.9)
    x = 0.9 * np.cos(angle_rad)
    y = 0.9 * np.sin(angle_rad)
    ax.plot([0, x], [0, y], color='red', linewidth=3)
    
    # Display the numeric value below the gauge
    ax.text(0, -0.25, f"{value:.2f}", ha='center', va='center', fontsize=10)
    ax.set_title(title, fontsize=10)

# ------------------
# Function: Read Serial Data and Update Buffers
# ------------------
def read_serial_data():
    global latest_data
    if ser.in_waiting > 0:
        try:
            # Decode with error replacement to avoid decoding issues
            line = ser.readline().decode('utf-8', errors='replace').strip()
            
            # Process only if it is valid telemetry data.
            if not is_valid_data(line):
                return
            
            # Log the raw line to file
            log_file.write(line + "\n")
            log_file.flush()
            
            # Convert the comma-separated values to a list of floats
            values = list(map(float, line.split(',')))
            latest_data = values  # Save the most recent valid telemetry
            
            # Update time series buffers:
            # Telemetry Altitude (index 2), Temperature (index 13), and Pressure (index 14)
            alt_ts.append(values[2])
            temp_ts.append(values[13])
            pres_ts.append(values[14])
        except Exception as e:
            print("Error parsing serial data:", e)

# ------------------
# Set Up the Figure Layout
# ------------------
fig = plt.figure(constrained_layout=True, figsize=(14, 10))
gs = fig.add_gridspec(3, 1)

# Top row: Time-series graphs arranged horizontally
ts_gs = gs[0].subgridspec(1, 3)
ax_ts_alt = fig.add_subplot(ts_gs[0])
ax_ts_temp = fig.add_subplot(ts_gs[1])
ax_ts_pres = fig.add_subplot(ts_gs[2])

# Middle row: 8 gauges arranged in a 2 x 4 grid
gauge_gs = gs[1].subgridspec(2, 4)
gauge_axes = [fig.add_subplot(gauge_gs[i//4, i%4]) for i in range(8)]

# Bottom row: Text panel for additional parameters (Gyroscope, Acceleration, Velocity)
ax_text = fig.add_subplot(gs[2])
ax_text.axis('off')

# ------------------
# Update Function for Animation
# ------------------
def update(frame):
    # Read new data from serial
    read_serial_data()
    
    # --- Update Time Series Graphs ---
    # Altitude (Telemetry value index 2)
    ax_ts_alt.clear()
    ax_ts_alt.plot(range(len(alt_ts)), alt_ts, 'b-', label="Altitude (m)")
    ax_ts_alt.set_title("Telemetry Altitude")
    ax_ts_alt.set_ylabel("Meters")
    ax_ts_alt.legend(fontsize=8)
    
    # Temperature (Telemetry value index 13)
    ax_ts_temp.clear()
    ax_ts_temp.plot(range(len(temp_ts)), temp_ts, 'r-', label="Temperature (°C)")
    ax_ts_temp.set_title("Temperature")
    ax_ts_temp.set_ylabel("°C")
    ax_ts_temp.legend(fontsize=8)
    
    # Pressure (Telemetry value index 14)
    ax_ts_pres.clear()
    ax_ts_pres.plot(range(len(pres_ts)), pres_ts, 'g-', label="Pressure")
    ax_ts_pres.set_title("Pressure")
    ax_ts_pres.set_ylabel("Pa")
    ax_ts_pres.legend(fontsize=8)
    
    # --- Update Gauges ---
    if latest_data is not None:
        # Gauge 1: Latitude (index 0, range -90 to 90)
        draw_gauge(gauge_axes[0], latest_data[0], -90, 90, "Latitude")
        
        # Gauge 2: Longitude (index 1, range -180 to 180)
        draw_gauge(gauge_axes[1], latest_data[1], -180, 180, "Longitude")
        
        # Gauge 3: GPS Altitude (index 15, range 0 to 5000)
        draw_gauge(gauge_axes[2], latest_data[15], 0, 5000, "GPS Altitude")
        
        # Gauge 4: Temperature (index 13, range -40 to 80 °C)
        draw_gauge(gauge_axes[3], latest_data[13], -40, 80, "Temperature")
        
        # Gauge 5: Pressure (index 14, assumed hPa, range 900 to 1100)
        draw_gauge(gauge_axes[4], latest_data[14], 900, 1100, "Pressure (hPa)")
        
        # Gauge 6: Humidity (index 12, range 0 to 100)
        draw_gauge(gauge_axes[5], latest_data[12], 0, 100, "Humidity")
        
        # Gauge 7: Number of Satellites (index 16, range 0 to 12)
        draw_gauge(gauge_axes[6], latest_data[16], 0, 12, "Satellites")
        
        # Gauge 8: Telemetry Altitude (index 2, range 0 to 5000)
        draw_gauge(gauge_axes[7], latest_data[2], 0, 5000, "Alt. (Telem)")
    
    # --- Update Text Panel for Additional Data ---
    if latest_data is not None:
        # Gyroscope: indices 3,4,5; Acceleration: indices 6,7,8; Velocity: indices 9,10,11
        gyro_str = f"Gyroscope (x, y, z): {latest_data[3]:.2f}, {latest_data[4]:.2f}, {latest_data[5]:.2f}"
        accel_str = f"Acceleration (x, y, z): {latest_data[6]:.2f}, {latest_data[7]:.2f}, {latest_data[8]:.2f}"
        vel_str   = f"Velocity (x, y, z): {latest_data[9]:.2f}, {latest_data[10]:.2f}, {latest_data[11]:.2f}"
    else:
        gyro_str = "Gyroscope: N/A"
        accel_str = "Acceleration: N/A"
        vel_str = "Velocity: N/A"
    
    text_message = "\n".join([gyro_str, accel_str, vel_str])
    ax_text.clear()
    ax_text.text(0.5, 0.5, text_message, fontsize=12, ha='center', va='center')
    ax_text.set_title("Other Telemetry Parameters")

# ------------------
# Set Up Animation
# ------------------
ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)

plt.show()

# ------------------
# Cleanup on Exit
# ------------------
log_file.close()
ser.close()
