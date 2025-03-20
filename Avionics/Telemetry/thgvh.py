import serial

# ------------------
# Configuration
# ------------------
SERIAL_PORT = "COM13"    # Update this port name (e.g., "COM5" on Windows or "/dev/ttyUSB0" on Linux/Mac)
BAUD_RATE = 115200

# ------------------
# Open Serial Port and Log File
# ------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

log_file = open("telemetry_log.txt", "a")

print("Logging data... Press Ctrl+C to stop.")

try:
    while True:
        # Read one line from the serial port, decoding with error replacement.
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            log_file.write(line + "\n")
            log_file.flush()
            print(line)  # Optionally print the data to the console
except KeyboardInterrupt:
    print("Logging stopped by user.")
except Exception as e:
    print("Error during logging:", e)
finally:
    log_file.close()
    ser.close()
