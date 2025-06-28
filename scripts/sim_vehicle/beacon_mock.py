import argparse
import socket
import struct
import sys
import json
import time
import signal
import sys

# Define constants
XYZ_AXIS_COUNT = 3
ADD_ANCHOR_HEADER = 0x00
REMOVE_ANCHOR_HEADER = 0x01
SAMPLE_HEADER = 0x02
RANGING_HEADER = 0x03
PREAMBLE = 0x55
DEFAULT_DELAY = 0.10  # Default delay between sample frames in seconds

anchors = {}           # Store anchor locations in a dictionary

# Define the RelativeLocation structure
class RelativeLocation:
    def __init__(self, x_m, y_m, z_m):
        self.x_m = x_m
        self.y_m = y_m
        self.z_m = z_m

    def pack(self):
        return struct.pack('fff', self.x_m, self.y_m, self.z_m)
# Define the AddAnchor frame
class AddAnchor:
    def __init__(self, anchor_id, location):
        self.preamble = PREAMBLE
        self.header = ADD_ANCHOR_HEADER
        self.anchor_id = anchor_id
        self.location = location
        self.crc = 0xFFFF  # Placeholder for CRC

    def pack(self):
        return struct.pack('BBB', self.preamble, self.header, self.anchor_id) + self.location.pack() + struct.pack('H', self.crc)

# Define the RemoveAnchor frame
class RemoveAnchor:
    def __init__(self, anchor_id):
        self.preamble = PREAMBLE
        self.header = REMOVE_ANCHOR_HEADER
        self.anchor_id = anchor_id
        self.crc = 0xFFFF  # Placeholder for CRC

    def pack(self):
        return struct.pack('BBBH', self.preamble, self.header, self.anchor_id, self.crc)

# Define the Sample frame with the updated structure
class Sample:
    def __init__(self, tag_loc, error_mm):
        self.preamble = PREAMBLE
        self.header = SAMPLE_HEADER  
        self.tag_loc = tag_loc
        self.error_mm = error_mm
        self.crc = 0xFFFF  # Placeholder for CRC

    def pack(self):
        # Correctly pack the preamble, header, tag location, error, and crc
        return struct.pack('BB', self.preamble, self.header) + self.tag_loc.pack() + struct.pack('HH', self.error_mm, self.crc)
    
class RangeSample:
    def __init__(self, anchor_id, range_m):
        self.preamble = PREAMBLE
        self.header = RANGING_HEADER
        self.anchor_id = anchor_id      # uint8_t 
        self.range_m = float(range_m)         # float
        self.crc = 0xFFFF  # Placeholder for CRC

    def pack(self):
        print(f"Ranging frame: {self.anchor_id}, {self.range_m}")
        return struct.pack('BBB', self.preamble, self.header, self.anchor_id) + struct.pack('f',self.range_m) + struct.pack('H',self.crc)

# Function to send a frame and receive ACK
def send_frame_and_receive_ack(frame, host, port):
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((host, port))
                s.sendall(frame.pack())
                # Receive ACK for AddAnchor and RemoveAnchor frames
                if isinstance(frame, (AddAnchor, RemoveAnchor)):
                    ack = s.recv(1)
                    if ack == struct.pack('B', frame.header):
                        print(f"ACK received for {type(frame).__name__}.")
                        break  # Exit the loop if ACK is received
                    else:
                        print(f"ACK not received for {type(frame).__name__}.")
                else:
                    print("No ACK needed for Sample frame.")
                    break  # Exit the loop if no ACK is needed
        except socket.error as e:
            print(f"Connection failed: {e}, retrying...")
            time.sleep(1)  # Wait for 5 seconds before retrying

# Function to read frames from a file
def read_frames_from_file(file_path):
    with open(file_path, 'r') as file:
        frames_data = json.load(file)
    return frames_data

# Function to create and send frames based on the data from the file
def create_and_send_frame(frame_data, host, port):
    frame_type = frame_data.get('frame_type')
    if frame_type == 'AddAnchor':
        anchor_id = frame_data.get('anchor_id')
        x_m = frame_data.get('x_m')
        y_m = frame_data.get('y_m')
        z_m = frame_data.get('z_m')
        location = RelativeLocation(x_m, y_m, z_m)
        frame = AddAnchor(anchor_id, location)
        # Store anchor location in the list if it is not already stored
        if anchor_id not in anchors:
            anchors[anchor_id] = location
    elif frame_type == 'RemoveAnchor':
        anchor_id = frame_data.get('anchor_id')
        frame = RemoveAnchor(anchor_id)
    elif frame_type == 'Sample':
        x_m = frame_data.get('x_m')
        y_m = frame_data.get('y_m')
        z_m = frame_data.get('z_m')
        location = RelativeLocation(x_m, y_m, z_m)
        error_mm = frame_data.get('error_mm')
        frame = Sample(location, error_mm)
        send_frame_and_receive_ack(frame, host, port)
        
        # For all stored anchors calculate and send ranging frames
        for anchor_id, anchor_location in anchors.items():
            range_m = float(((anchor_location.x_m - x_m) ** 2 + (anchor_location.y_m - y_m) ** 2 + (anchor_location.z_m - z_m) ** 2) ** 0.5)
            frame = RangeSample(anchor_id, range_m)
            send_frame_and_receive_ack(frame, host, port)
        return # Exit the function after sending ranging frames
    else:
        print(f'Invalid frame type: {frame_type}')
        return

    send_frame_and_receive_ack(frame, host, port)

# Function to handle CTRL+C
def signal_handler(sig, frame):
    print('Exiting loop...')
    sys.exit(0)

# Main execution
if __name__ == '__main__':
    # Parse command line arguments
    # Argument parser setup
    parser = argparse.ArgumentParser(description='KonexUWBFrame Command-Line Tool')
    subparsers = parser.add_subparsers(dest='frame_type', help='Type of frame to send')

    # Sub-command for AddAnchor
    add_anchor_parser = subparsers.add_parser('AddAnchor', help='Send an AddAnchor frame')
    add_anchor_parser.add_argument('anchor_id', type=int, help='Anchor ID')
    add_anchor_parser.add_argument('x_m', type=float, help='X coordinate in meters')
    add_anchor_parser.add_argument('y_m', type=float, help='Y coordinate in meters')
    add_anchor_parser.add_argument('z_m', type=float, help='Z coordinate in meters')

    # Sub-command for RemoveAnchor
    remove_anchor_parser = subparsers.add_parser('RemoveAnchor', help='Send a RemoveAnchor frame')
    remove_anchor_parser.add_argument('anchor_id', type=int, help='Anchor ID')

    # Sub-command for Sample
    sample_parser = subparsers.add_parser('Sample', help='Send a Sample frame')
    sample_parser.add_argument('tag_position_cm', nargs=3, type=int, help='Tag position in cm (x, y, z)')
    sample_parser.add_argument('error_mm', type=int, help='Error in mm')

    # Common arguments
    parser.add_argument('--host', type=str, default='localhost', help='Host to connect to (default: localhost)')
    parser.add_argument('--port', type=int, default=5765, help='Port to connect to (default: 5765)')
    parser.add_argument('--file', type=str, help='Path to the file containing the sequence of frames to send')
    parser.add_argument('--loop', action='store_true', help='Enable looping through sample frames')
    parser.add_argument('--loop-frames', nargs='+', help='Frame types to send in loop (e.g., Sample AddAnchor)')

    # Parse the arguments
    args = parser.parse_args()

    # Register signal handler for CTRL+C
    signal.signal(signal.SIGINT, signal_handler)

    # Check if a file is provided
    if args.file:
        frames_data = read_frames_from_file(args.file)
        loop_frames = set(args.loop_frames) if args.loop_frames else set()

        # Send all frames once
        for frame_data in frames_data:
            if frame_data['frame_type'] not in loop_frames:
                create_and_send_frame(frame_data, args.host, args.port)

        # Check if loop option is enabled
        if args.loop:
            try:
                while True:
                    for frame_data in frames_data:
                        if frame_data['frame_type'] in loop_frames:
                            create_and_send_frame(frame_data, args.host, args.port)
                            time.sleep(DEFAULT_DELAY)
            except KeyboardInterrupt:
                print('Loop interrupted by user.')
                sys.exit(0)
    else:
        # Handle command-line arguments for individual frame sending
        if args.frame_type is None:
            parser.print_help()
            sys.exit(1)

        # Handle each frame type
        if args.frame_type == 'AddAnchor':
            location = RelativeLocation(args.x_m, args.y_m, args.z_m)
            frame = AddAnchor(args.anchor_id, location)
        elif args.frame_type == 'RemoveAnchor':
            frame = RemoveAnchor(args.anchor_id)
        elif args.frame_type == 'Sample':
            frame = Sample(args.tag_position_cm, args.error_mm)
        else:
            print('Invalid frame type')
            sys.exit(1)

        # Send the frame
        send_frame_and_receive_ack(frame, args.host, args.port)
