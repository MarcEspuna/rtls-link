import argparse
import websocket
import threading

def on_message(ws, message):
    print(message + "\n> ", end='')

def on_error(ws, error):
    print("Error: " + str(error))

def on_close(ws, close_status_code, close_msg):
    print("### closed ###")

def on_open(ws):
    def run(*args):
        print("---- Connected to WebSocket server ----")
        print("> ", end='')  # Display the prompt on the same line
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
    args = parser.parse_args()

    if args.verbose:
        websocket.enableTrace(True)
    ws = websocket.WebSocketApp(f"ws://{args.host}:{args.port}/ws",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    ws.run_forever()

