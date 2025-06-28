import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import numpy as np
from itertools import product, combinations

# Initialize the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set the maximum axis values for the plot (customize as needed)
max_axis_values = (5, 5, 3)  # Room size (width, length, height)
ax.set_xlim(0, max_axis_values[0])
ax.set_ylim(0, max_axis_values[1])
ax.set_zlim(0, max_axis_values[2])

# Draw a box representing the room
r = [0, max_axis_values[0]]  # X axis range
s = [0, max_axis_values[1]]  # Y axis range
t = [0, max_axis_values[2]]  # Z axis range
for s, e in combinations(np.array(list(product(r, s, t))), 2):
    if np.sum(np.abs(s - e)) == r[1] - r[0] or np.sum(np.abs(s - e)) == s[1] - s[0] or np.sum(np.abs(s - e)) == t[1] - t[0]:
        ax.plot3D(*zip(s, e), color="b")

# Store the last dot's reference
last_dot = None

# Function to update the plot with new data
def update_plot(x, y, z, last_dot):
    global ax
    # Remove the last dot from the plot
    if last_dot:
        last_dot.remove()
    # Plot the new dot
    new_dot, = ax.plot([x], [y], [z], 'ro')
    plt.draw()
    plt.pause(0.01)
    return new_dot

# Function to listen to the COM port
def listen_to_com_port(com_port, baudrate):
    global last_dot
    with serial.Serial(com_port, baudrate, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                # Print all data received from the COM port
                print(line)
                # Check if the line matches the expected format
                match = re.match(r'P= (-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)', line)
                if match:
                    # Extract the x, y, z coordinates
                    x, y, z = map(float, match.groups())
                    # Update the plot with the new coordinates
                    last_dot = update_plot(x, y, z, last_dot)
            except serial.SerialException as e:
                print(f"Serial exception: {e}")
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")

# Specify the COM port and baudrate (customize as needed)
com_port = 'COM14'
baudrate = 115200

# Start listening to the COM port
listen_to_com_port(com_port, baudrate)

# Note: Ensure that the pySerial package is installed in your Python environment.
# You can install it using the following command: pip install pyserial
