import serial
import time
import keyboard # type: ignore

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

def send_rc():
    rc = f"RC,{roll},{pitch},{throttle},{yaw}"
    ser.write(rc.encode())
    print(rc.strip())

try:
    while True:
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

        # Send RC command
        send_rc() #250Hz
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()


# Run this script with Python 3 and ensure you have the 'keyboard' library installed.
# You can install it using: pip install keyboard

# sudo ~/drone-rc-venv/bin/python teclado.py #