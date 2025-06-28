import socket
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import re
import numpy as np
from itertools import product, combinations
import threading
from queue import Queue
import json

# Queue for thread communication
data_queue = Queue()

def load_config(config_file):
    with open(config_file, 'r') as f:
        return json.load(f)

def setup_plot(config):
    fig = plt.figure()
    if config['mode'] == '3D':
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(config['north_min'], config['north_max'])
        ax.set_ylim(config['east_min'], config['east_max'])
        ax.set_zlim(config['down_min'], config['down_max'])
        ax.set_xlabel('North')
        ax.set_ylabel('East')
        ax.set_zlabel('Down')
        draw_3d_box(ax, config)
    else:  # 2D mode
        ax = fig.add_subplot(111)
        ax.set_xlim(config['east_min'], config['east_max'])
        ax.set_ylim(config['north_min'], config['north_max'])
        ax.set_xlabel('East')
        ax.set_ylabel('North')
        ax.invert_yaxis()  # Invert y-axis to make North point up
        draw_2d_box(ax, config)
    return fig, ax

def draw_3d_box(ax, config):
    r = [config['north_min'], config['north_max']]
    s = [config['east_min'], config['east_max']]
    t = [config['down_min'], config['down_max']]
    for s, e in combinations(np.array(list(product(r, s, t))), 2):
        if np.sum(np.abs(s - e)) == r[1] - r[0] or np.sum(np.abs(s - e)) == s[1] - s[0] or np.sum(np.abs(s - e)) == t[1] - t[0]:
            ax.plot3D(*zip(s, e), color="b")

def draw_2d_box(ax, config):
    ax.axhline(y=config['north_min'], color='b', linestyle='-')
    ax.axhline(y=config['north_max'], color='b', linestyle='-')
    ax.axvline(x=config['east_min'], color='b', linestyle='-')
    ax.axvline(x=config['east_max'], color='b', linestyle='-')

def update_plot(ax, north, east, down, last_dot, config):
    if last_dot:
        last_dot.remove()
    if config['mode'] == '3D':
        new_dot, = ax.plot([north], [east], [down], 'ro')
    else:
        new_dot, = ax.plot([east], [north], 'ro')
    plt.draw()
    plt.pause(0.001)
    return new_dot

def extract_data(data, config):
    pattern = r"(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+hz"
    match = re.match(pattern, data)
    if match:
        try:
            north, east, down = map(float, match.groups()[:3])
            refresh_rate = int(match.group(4))
            return (north, east, down, refresh_rate) if config['mode'] == '3D' else (north, east, refresh_rate)
        except (ValueError, IndexError):
            print("Error: Data extraction failed. Check the input data format.")
            return None
    else:
        print("Error: No data match found in the input.")
        return None

def socket_thread_func(ip, port, config):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, port)
    print(f'Connecting to {ip} port {port}')
    sock.connect(server_address)

    try:
        while True:
            data = sock.recv(100).decode()
            if not data:
                print('No more data from', server_address)
                break
            print(data)
            point_data = extract_data(data, config)
            if point_data:
                data_queue.put(point_data)
    finally:
        print('Closing socket')
        sock.close()

def main(ip, port, config_file):
    config = load_config(config_file)
    fig, ax = setup_plot(config)
    last_dot = None

    socket_thread = threading.Thread(target=socket_thread_func, args=(ip, port, config))
    socket_thread.daemon = True
    socket_thread.start()

    while True:
        if not data_queue.empty():
            point_data = data_queue.get()
            if config['mode'] == '3D':
                north, east, down, refresh_rate = point_data
                last_dot = update_plot(ax, north, east, down, last_dot, config)
            else:
                north, east, refresh_rate = point_data
                last_dot = update_plot(ax, north, east, None, last_dot, config)
        else:
            plt.pause(0.01)

if __name__ == '__main__':
    plt.ion()
    if len(sys.argv) != 4:
        print('Usage: python script.py <IP> <Port> <ConfigFile>')
        sys.exit(1)

    ip_arg = sys.argv[1]
    port_arg = int(sys.argv[2])
    config_file = sys.argv[3]
    
    main(ip_arg, port_arg, config_file)