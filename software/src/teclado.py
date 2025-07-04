import serial
import time
import keyboard # type: ignore
import threading
import sys

# Connect to your RP2040 COM port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

# Constants
NEUTRAL = 1500
HIGH = 1900
LOW = 1100
THROTTLE_ARM = 1100
THROTTLE_HOVER = 1500
THROTTLE_CLIMB = 2000
THROTTLE_DESCEND = 1000
THROTTLE_MIN = 1000
THROTTLE_MAX = 1800

# Emergency reset delay
EMERGENCY_DELAY = 0.3

# Initialize RC values
roll = pitch = yaw = NEUTRAL
throttle = THROTTLE_HOVER

# Control flags
running = True
logging_active = False

def send_rc():
    rc = f"RC,{roll},{pitch},{throttle},{yaw}"
    ser.write(rc.encode())
    print(rc.strip())

def send_special_command(command):
    """Send special command to docking station"""
    ser.write(f"{command}\n".encode())
    print(f"Sent: {command}")

#def read_telemetry():
#    """Read telemetry data from drone"""
#    global running
#    while running:
#        try:
#            if ser.in_waiting > 0:
#                data = ser.readline().decode().strip()
#                if data.startswith("TELEM,"):
#                    # Parse and display telemetry (optional)
#                    pass
#                elif data:
#                    print(f"Received: {data}")
#        except Exception as e:
#            print(f"Error reading telemetry: {e}")
#        time.sleep(0.01)

def show_controls():
    """Display control instructions"""
    print("\nDrone Controller Active")
    print("Controls:")
    print("  W/S: Pitch forward/backward")
    print("  A/D: Roll left/right")
    print("  Q/E: Yaw left/right")
    print("  U/J: Throttle up/down")
    print("  P: Arm throttle")
    print("  X: Emergency stop")
    print("  L: Toggle logging")
    print("  I: Show current values")
    print("  H: Show this help")
    print("  ESC: Exit")
    print("\nPress keys to control drone...")

# Start telemetry reader thread
#telemetry_thread = threading.Thread(target=read_telemetry)
#telemetry_thread.daemon = True
#telemetry_thread.start()

# Show controls at startup
show_controls()

try:
    while running:
        # Reset to neutral every frame
        roll = pitch = yaw = NEUTRAL
        throttle = THROTTLE_HOVER

        # Movement controls
        if keyboard.is_pressed('w'):
            pitch = HIGH
        if keyboard.is_pressed('s'):
            pitch = LOW
        if keyboard.is_pressed('a'):
            roll = LOW
        if keyboard.is_pressed('d'):
            roll = HIGH
        if keyboard.is_pressed('q'):
            yaw = LOW
        if keyboard.is_pressed('e'):
            yaw = HIGH

        # Throttle control like joystick
        if keyboard.is_pressed('p'):
            throttle = THROTTLE_ARM
        if keyboard.is_pressed('u'):
            throttle = THROTTLE_CLIMB
        elif keyboard.is_pressed('j'):
            throttle = THROTTLE_DESCEND

        # Emergency stop (everything neutral + throttle min)
        if keyboard.is_pressed('x'):
            roll = pitch = yaw = NEUTRAL
            throttle = THROTTLE_MIN
            print("EMERGENCY STOP")
            send_rc()
            time.sleep(EMERGENCY_DELAY)
            continue

        # New features from DroneController
        if keyboard.is_pressed('l'):
            send_special_command("log")
            logging_active = not logging_active
            print(f"Logging {'started' if logging_active else 'stopped'}")
            time.sleep(0.5)

        if keyboard.is_pressed('i'):
            print(f"R:{roll} P:{pitch} T:{throttle} Y:{yaw}")
            time.sleep(0.2)

        if keyboard.is_pressed('h'):
            show_controls()
            time.sleep(0.5)

        # Send RC command
        send_rc()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    running = False
    ser.close()
    print("Controller stopped")

# Run this script with Python 3 and ensure you have the 'keyboard' library installed.
# You can install it using: pip install keyboard

# sudo ~/drone-rc-venv/bin/python teclado.py