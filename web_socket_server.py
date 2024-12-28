import asyncio
import websockets
import cv2
import numpy as np

# Dictionary to store video frames by client IP
video_frames = {}

# Window name for display
WINDOW_NAME = "4 Streams Display"

async def handle_camera_connection(websocket, path):
    client_ip = websocket.remote_address[0]
    print(f"Connection from: {client_ip}")

    try:
        async for message in websocket:
            if isinstance(message, bytes):  # Binary frame data
                # Decode the received frame
                frame_array = np.frombuffer(message, dtype=np.uint8)
                frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                # Store the latest frame for this client
                video_frames[client_ip] = frame
            else:
                print(f"Received non-binary message from {client_ip}: {message}")
    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed for {client_ip}")
        video_frames.pop(client_ip, None)

async def display_frames():
    while True:
        # Prepare a black canvas to display 4 streams (2x2 grid)
        canvas = np.zeros((480, 640, 3), dtype=np.uint8)

        # Get frames from clients
        frames = list(video_frames.values())

        # Resize and arrange up to 4 frames in the canvas
        for i, frame in enumerate(frames[:4]):
            resized_frame = cv2.resize(frame, (320, 240))  # Resize to fit 2x2 grid

            # Calculate position in the grid
            x_offset = (i % 2) * 320
            y_offset = (i // 2) * 240
            canvas[y_offset:y_offset+240, x_offset:x_offset+320] = resized_frame

        # Show the canvas
        cv2.imshow(WINDOW_NAME, canvas)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        await asyncio.sleep(0.03)  # Approx. 30 FPS

async def main():
    # Start the WebSocket server
    websocket_server = websockets.serve(handle_camera_connection, "0.0.0.0", 8080)
    print("WebSocket server running on ws://0.0.0.0:8080")

    # Run server and frame display concurrently
    await asyncio.gather(
        websocket_server,
        display_frames()
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cv2.destroyAllWindows()
