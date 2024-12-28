import aiohttp
from aiohttp import web
import asyncio
import websockets
import cv2
import numpy as np

# Dictionary to store video frames by client IP
video_frames = {}

# Window name for display
WINDOW_NAME = "4 Streams Display"

async def index(request):
    return web.Response(text=open('index.html').read(), content_type='text/html')

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    # Handle incoming messages
    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            if msg.data == 'close':
                await ws.close()
            else:
                await ws.send_str(f"Message received: {msg.data}")
        elif msg.type == aiohttp.WSMsgType.ERROR:
            print(f"WebSocket connection closed with exception {ws.exception()}")

    return ws

async def handle_camera_connection(websocket):
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

                # Example: Send a response back to the client
                # response = f"Frame received from {client_ip}"
                # await websocket.send(response)
            else:
                print(f"Received non-binary message from {client_ip}: {message}")
    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed for {client_ip}")
        video_frames.pop(client_ip, None)

async def display_frames():
    while True:
        # Prepare a black canvas to display 4 streams (2x2 grid)
        canvas = np.zeros((960, 1280, 3), dtype=np.uint8)

        # Get frames from clients
        frames = list(video_frames.items())

        # Resize and arrange up to 4 frames in the canvas
        for i, (ip, frame) in enumerate(frames[:4]):
            resized_frame = cv2.resize(frame, (640, 480))  # Resize to fit 2x2 grid

            # Calculate position in the grid
            x_offset = (i % 2) * 640
            y_offset = (i // 2) * 480
            canvas[y_offset:y_offset+480, x_offset:x_offset+640] = resized_frame

            # Add the IP address text to the frame
            text_position = (x_offset + 10, y_offset + 30)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_color = (255, 255, 255)
            thickness = 2
            cv2.putText(canvas, ip, text_position, font, font_scale, font_color, thickness)

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

app = web.Application()
app.router.add_get('/', index)
app.router.add_get('/ws', websocket_handler)

web.run_app(app, host='localhost', port=8080)

# if __name__ == "__main__":
#    try:
#        asyncio.run(main())
#    finally:
#        cv2.destroyAllWindows()
