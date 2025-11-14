import serial
import time
import re
import os

def get_terminal_width():
    """Gets the width of the terminal."""
    try:
        return os.get_terminal_size().columns
    except OSError:
        # A default width if the terminal size cannot be determined
        return 80

def visualize_channel(channel_name, value, min_val=1000, max_val=2000):
    """
    Creates a simple text-based visualization for a single channel.

    Args:
        channel_name (str): The name of the channel (e.g., "THROTTLE").
        value (int): The current value of the channel.
        min_val (int): The minimum expected value for the channel.
        max_val (int): The maximum expected value for the channel.
    """
    terminal_width = get_terminal_width()
    # Reserve space for the label and value display
    label_and_value = f"{channel_name}: {value} "
    bar_max_width = terminal_width - len(label_and_value) - 2 # -2 for '|' characters

    # Clamp the value within the expected range
    value = max(min_val, min(value, max_val))

    # Scale the value to the bar width
    bar_length = int(((value - min_val) / (max_val - min_val)) * bar_max_width)
    bar = '█' * bar_length
    padding = ' ' * (bar_max_width - bar_length)

    print(f"{label_and_value}|{bar}{padding}|")

def read_and_visualize_drone_channels(serial_port, baud_rate=115200):
    """
    Reads drone remote control data from a UART port, parses it,
    and visualizes the channel data in the command line.

    Args:
        serial_port (str): The name of the serial port 
                           (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux).
        baud_rate (int): The baud rate for the serial communication.
    """
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Listening on {serial_port} at {baud_rate} bps...")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {serial_port}: {e}")
        return

    # This is the new, more specific regex pattern
    # It looks for the exact "KEY = VALUE" structure for each channel
    pattern = re.compile(
        r"THROTTLE\s*=\s*(\d+)\s*\|\s*ROLL\s*=\s*(\d+)\s*\|\s*YAW\s*=\s*(\d+)\s*\|\s*PITCH\s*=\s*(\d+)"
    )

    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            try:
                line_str = line.decode('utf-8').strip()
            except UnicodeDecodeError:
                continue

            # Search for the specific pattern in the line
            match = pattern.search(line_str)
            
            # If the pattern is found, match will not be None
            if match:
                try:
                    # The captured groups (the numbers) are extracted
                    throttle, roll, yaw, pitch = map(int, match.groups())

                    os.system('cls' if os.name == 'nt' else 'clear')

                    print("--- Drone Remote Control Channels ---")
                    # Using a slightly different min/max to better fit your example data
                    visualize_channel("THROTTLE", throttle, min_val=0, max_val=2000)
                    visualize_channel("ROLL    ", roll, min_val=0, max_val=2000)
                    visualize_channel("YAW     ", yaw, min_val=0, max_val=2000)
                    visualize_channel("PITCH   ", pitch, min_val=0, max_val=2000)
                    print("------------------------------------")

                except (ValueError, IndexError):
                    # This will now be much less likely to happen
                    print(f"Could not parse data: {line_str}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping visualization.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"Serial port {serial_port} closed.")

if __name__ == '__main__':
    # --- Configuration ---
    # Change this to your serial port
    SERIAL_PORT = '/dev/ttyUSB0'  # For Linux, or 'COM3' for Windows
    BAUD_RATE = 115200

    read_and_visualize_drone_channels(SERIAL_PORT, BAUD_RATE)