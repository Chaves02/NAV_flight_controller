import serial
import time
import keyboard # type: ignore
import threading
import sys
import os
from datetime import datetime

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

# CSV capture variables
csv_file = None
csv_filename = None
csv_logging = False
telemetry_count = 0
csv_header_written = False

def send_rc():
    """Send RC command to drone"""
    rc = f"RC,{roll},{pitch},{throttle},{yaw}\n"
    ser.write(rc.encode())
    print(f"RC: R:{roll} P:{pitch} T:{throttle} Y:{yaw}")

def send_special_command(command):
    """Send special command to docking station"""
    ser.write(f"{command}\n".encode())
    print(f"Sent: {command}")

def start_csv_logging():
    """Start CSV logging to file"""
    global csv_file, csv_filename, csv_logging, telemetry_count, csv_header_written
    
    if csv_logging:
        print("CSV logging already active")
        return
    
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"telemetry_{timestamp}.csv"
        csv_file = open(csv_filename, 'w')
        csv_logging = True
        telemetry_count = 0
        csv_header_written = False
        print(f"Started CSV logging to: {csv_filename}")
        return True
    except Exception as e:
        print(f"Failed to start CSV logging: {e}")
        return False

def stop_csv_logging():
    """Stop CSV logging and close file"""
    global csv_file, csv_logging, telemetry_count
    
    if not csv_logging:
        print("CSV logging not active")
        return
    
    try:
        if csv_file:
            csv_file.close()
            csv_file = None
        csv_logging = False
        print(f"Stopped CSV logging. Saved {telemetry_count} records to {csv_filename}")
    except Exception as e:
        print(f"Error stopping CSV logging: {e}")

def process_telemetry_line(line):
    """Process incoming telemetry data"""
    global csv_file, csv_logging, telemetry_count, csv_header_written
    
    line = line.strip()
    
    if line.startswith("CSV_START:"):
        # Pico started logging
        if not csv_logging:
            start_csv_logging()
        print(f"Pico started logging: {line}")
    
    elif line.startswith("CSV_HEADER:"):
        # Write CSV header
        if csv_logging and csv_file and not csv_header_written:
            header = line[11:].strip()  # Remove "CSV_HEADER:" prefix
            csv_file.write(header + "\n")
            csv_file.flush()
            csv_header_written = True
            print(f"CSV Header written: {header}")
    
    elif line.startswith("CSV_DATA:"):
        # Write CSV data
        if csv_logging and csv_file:
            data = line[9:].strip()  # Remove "CSV_DATA:" prefix
            csv_file.write(data + "\n")
            csv_file.flush()
            telemetry_count += 1
            
            # Print every 10th record to avoid spam
            if telemetry_count % 10 == 0:
                print(f"CSV Data #{telemetry_count}: {data}")
    
    elif line.startswith("CSV_STOP:"):
        # Pico stopped logging
        print(f"Pico stopped logging: {line}")
        if csv_logging:
            stop_csv_logging()
    
    elif line.startswith("TELEM:"):
        # Regular telemetry output (display but don't log)
        print(f"Telemetry: {line}")
    
    elif line.startswith("Logging:"):
        # Status response
        print(f"Status: {line}")
    
    else:
        # Other console output from Pico
        if line and not line.startswith("RC:"):  # Don't echo our own RC commands
            print(f"Pico: {line}")

def serial_reader():
    """Background thread to read serial data"""
    global running
    
    while running:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore')
                if line:
                    process_telemetry_line(line)
        except Exception as e:
            print(f"Error reading serial data: {e}")
        
        time.sleep(0.01)  # Small delay to prevent high CPU usage

def show_controls():
    """Display control instructions"""
    print("\n" + "="*60)
    print("INTEGRATED DRONE CONTROLLER WITH CSV CAPTURE")
    print("="*60)
    print("DRONE CONTROLS:")
    print("  W/S: Pitch forward/backward")
    print("  A/D: Roll left/right")
    print("  Q/E: Yaw left/right")
    print("  U/J: Throttle up/down")
    print("  P: Arm throttle")
    print("  X: Emergency stop")
    print()
    print("TELEMETRY & LOGGING:")
    print("  L: Toggle logging on Pico")
    print("  C: Start/Stop CSV capture")
    print("  O: Show status")
    print("  T: Show telemetry status")
    print()
    print("GENERAL:")
    print("  I: Show current RC values")
    print("  H: Show this help")
    print("  ESC: Exit")
    print("="*60)
    print("Press keys to control drone...")

def show_status():
    """Show current status"""
    print("\n" + "-"*40)
    print("CURRENT STATUS:")
    print(f"  RC Values: R:{roll} P:{pitch} T:{throttle} Y:{yaw}")
    print(f"  Pico Logging: {'ON' if logging_active else 'OFF'}")
    print(f"  CSV Capture: {'ON' if csv_logging else 'OFF'}")
    if csv_logging:
        print(f"  CSV File: {csv_filename}")
        print(f"  Records: {telemetry_count}")
    print("-"*40)

# Start background serial reader thread
serial_thread = threading.Thread(target=serial_reader, daemon=True)
serial_thread.start()

# Show controls at startup
show_controls()

try:
    last_key_time = {}  # Track last key press times to prevent spam
    
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
            print("  EMERGENCY STOP ")
            send_rc()
            time.sleep(EMERGENCY_DELAY)
            continue

        # Prevent key spam with timing
        current_time = time.time()
        
        # Toggle Pico logging
        if keyboard.is_pressed('l'):
            if current_time - last_key_time.get('l', 0) > 0.5:
                send_special_command("log")
                logging_active = not logging_active
                print(f"  Pico logging {'started' if logging_active else 'stopped'}")
                last_key_time['l'] = current_time

        # Toggle CSV capture
        if keyboard.is_pressed('c'):
            if current_time - last_key_time.get('c', 0) > 0.5:
                if csv_logging:
                    stop_csv_logging()
                else:
                    start_csv_logging()
                last_key_time['c'] = current_time

        # Show current values
        if keyboard.is_pressed('i'):
            if current_time - last_key_time.get('i', 0) > 0.5:
                show_status()
                last_key_time['i'] = current_time

        # Show status
        if keyboard.is_pressed('o'):
            if current_time - last_key_time.get('o', 0) > 0.5:
                send_special_command("status")
                show_status()
                last_key_time['o'] = current_time

        # Show telemetry status
        if keyboard.is_pressed('t'):
            if current_time - last_key_time.get('t', 0) > 0.5:
                print(f"  Telemetry Status:")
                print(f"  CSV Logging: {'Active' if csv_logging else 'Inactive'}")
                print(f"  Records Captured: {telemetry_count}")
                if csv_logging:
                    print(f"  Current File: {csv_filename}")
                last_key_time['t'] = current_time

        # Show help
        if keyboard.is_pressed('h'):
            if current_time - last_key_time.get('h', 0) > 0.5:
                show_controls()
                last_key_time['h'] = current_time

        # Exit
        if keyboard.is_pressed('esc'):
            print("ESC pressed - exiting...")
            break

        # Send RC command
        send_rc()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n Keyboard interrupt received...")
except Exception as e:
    print(f"Error: {e}")
finally:
    running = False
    
    # Clean up
    if csv_logging:
        stop_csv_logging()
    
    try:
        ser.close()
        print("Serial connection closed")
    except:
        pass
    
    print("Controller stopped")

# Installation requirements:
# pip install keyboard pyserial
# Run with: sudo ~/drone-rc-venv/bin/python integrated_controller.py