import argparse
import websocket
import threading

def on_message(ws, message):
    print(message + "\n> ", end='')

def on_error(ws, error):
    print("Error: " + str(error))

def on_close(ws, close_status_code, close_msg):
    print("### closed ###")

def on_open(ws, args):
    def run():
        print("---- Connected to WebSocket server ----")
        if args.file:
            with open(args.file, 'r') as f:
                for line in f:
                    parts = line.strip().split(': ')
                    if len(parts) == 2:
                        group, rest = parts[0].split('.')
                        name, data = rest, parts[1]
                        command = f"write -group {group} -name {name} -data {data}"
                        ws.send(command)
                        print(f"Sent: {command}")
        while True:
            try:
                message = input()
                if message == "exit":
                    ws.close()
                    break
                ws.send(message)
            except KeyboardInterrupt:
                print("\nAttempting to close WebSocket connection...")
                ws.close()
                break

    thread = threading.Thread(target=run)
    thread.start()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='WebSocket Remote Terminal')
    parser.add_argument('--host', type=str, default='localhost', help='Host to connect to')
    parser.add_argument('--port', type=int, default=8000, help='Port to connect to')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('-f', '--file', type=str, help='File containing parameters to upload')
    args = parser.parse_args()

    if args.verbose:
        websocket.enableTrace(True)
    ws = websocket.WebSocketApp(f"ws://{args.host}:{args.port}/ws",
                                on_open=lambda ws: on_open(ws, args),
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    ws.run_forever()
