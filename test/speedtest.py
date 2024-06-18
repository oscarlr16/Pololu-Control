import serial
import time
import threading

# Configuration for the serial port
port = 'COM4'
baudrate = 115200
ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(2)  # Wait for the connection to establish

# Global variables to control the motion and interval
position = 0
target = 800
direction = 1  # Initial direction
total_travel_time = 10.0  # Default total travel time in seconds, adjustable by user
change_time = False

def move_to_position():
    global position, total_travel_time, change_time, target, direction
    steps = 800  # Since we are moving from 0 to 800
    interval = total_travel_time / steps
    
    while True:
        # Move to the next position
        ser.write(f'moveto,{position}\n'.encode())
        print(f'Sent: moveto,{position}')
        time.sleep(interval)
        
        # Update position
        if position == target:
            direction = -direction  # Switch direction at target
            target = 0 if target == 800 else 800
        
        position += direction
        
        if position == 0 and change_time:  # Apply new travel time at position 0
            interval = total_travel_time / steps
            change_time = False

def listen_for_time_change():
    global total_travel_time, change_time
    while True:
        new_travel_time = input("Enter new total travel time in seconds for 0 to 800: ")
        try:
            total_travel_time = float(new_travel_time)
            change_time = True
            print(f"Total travel time will change to {total_travel_time} seconds when it reaches 0.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")

# Start the serial communication in a separate thread
threading.Thread(target=move_to_position, daemon=True).start()

# Start listening for travel time changes
listen_for_time_change()
