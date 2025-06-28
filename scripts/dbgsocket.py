import socket
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

# Function to extract point and refresh rate from the received data
def extract_data(data):
  pattern = r"(X|Y|Z): ([-+]?\d*\.\d+|\d+), ?"  # Match X, Y, Z followed by numbers (optional sign, digits, dot, digits)
  matches = re.findall(pattern, data)
  if matches:
    # Extract data from each match (assuming all lines have same format)
    x, y, z = [float(val[1]) for val in matches if val[0] == 'X'],  \
               [float(val[1]) for val in matches if val[0] == 'Y'],  \
               [float(val[1]) for val in matches if val[0] == 'Z']
    refresh_rate = int(data.split("refresh ")[-1].split(" hz")[0])  # Separate refresh rate value
    return x[0], y[0], z[0], refresh_rate
  else:
    return None

# Store the last dot's reference
last_dot = None

# Function to update the plot with the new point
def update_plot(ax, point, keep_trace=False):
  if not keep_trace:
    ax.cla()  # Clear Axes instead of scatter points
  ax.set_xlim(0, 3)  # Set limits for the plot (considering 8x8x8 box)
  ax.set_ylim(0, 3)
  ax.set_zlim(0, 2)
  ax.scatter(point[0], point[1], point[2], c='r', marker='o')
  plt.draw()
  plt.pause(0.01)  # Pause for the refresh rate


def main(ip, port):
  # Create a TCP/IP socket
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  # Connect the socket to the server's address and port
  server_address = (ip, port)
  print(f'Connecting to {ip} port {port}')
  sock.connect(server_address)

  # Set up the plot
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  plt.ion()
  plt.show()

  try:
    while True:
      data = sock.recv(100).decode()
      if not data:
        print('No more data from', server_address)
        break
      print(data)
      point_data = extract_data(data)
      if point_data:
        point, refresh_rate = point_data[:3], point_data[3]
        print(f"Point: {point}, Refresh rate: {refresh_rate} Hz")
        update_plot(ax, point)
  finally:
    # Clean up the connection
    print('Closing socket')
    sock.close()
    plt.ioff()
    plt.show()

if __name__ == '__main__':
  if len(sys.argv) != 3:
    print('Usage: python script.py <IP> <Port>')
    sys.exit(1)

  ip_arg = sys.argv[1]
  port_arg = int(sys.argv[2])
  main(ip_arg, port_arg)
