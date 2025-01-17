import logging

import time
import aiohttp
from aiohttp import web
import asyncio
import websockets
import cv2
import numpy as np
import json

frame_lock = asyncio.Lock()
control_lock = asyncio.Lock()

# Window name for display
WINDOW_NAME = "4 Streams Display"

async def index(request):
    return web.Response(text=open('index.html').read(), content_type='text/html')

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    client_ip = request.remote

    # Handle incoming messages
    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            if msg.data == 'close':
                await ws.close()
            else:
                # await ws.send_str(f"Message received: {msg.data}")
                # print(f"Message received: {msg.data}")
                async with control_lock:
                    request.app['control_commands'].clear()
                    request.app['control_commands'].extend(json.loads(msg.data))
        elif msg.type == aiohttp.WSMsgType.BINARY:
            # Decode the received frame
            frame_array = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
            # Store the latest frame for this client
            async with frame_lock:
                request.app['video_frames'][client_ip] = frame

            ip_addresses = list(request.app['video_frames'].keys())
            ip_idx = ip_addresses.index(client_ip)

            try:
                async with control_lock:
                    command = request.app['control_commands'][ip_idx]
                # print(f"CONTROL:{client_ip}:{command[0]}:{command[1]}")
                await ws.send_str(f"CONTROL:{command[0]}:{command[1]}")
            except IndexError:
                print(f"Index {ip_idx} does not exist.")
        elif msg.type == aiohttp.WSMsgType.ERROR:
            print(f"WebSocket connection closed with exception {ws.exception()}")

    return ws


async def generate_frames(request):
    while True:
        # Prepare a black canvas to display 4 streams (2x2 grid)
        canvas = np.zeros((960, 1280, 3), dtype=np.uint8)

        # Get frames from clients

        async with frame_lock:
            frames = list(request.app['video_frames'].items())

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

        _, jpeg_frame = cv2.imencode('.jpg', canvas)
        frame_data = jpeg_frame.tobytes()

        yield frame_data
        await asyncio.sleep(1/30) # FPS

async def stream_video(request):
    try:
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-cache',
                'Connection': 'keep-alive'
            }
        )
        await response.prepare(request)

        # Get frames from the generator
        frame_generator = generate_frames(request)

        # Stream the frames to the client
        async for frame_data in frame_generator:
            await response.write(
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n\r\n'
            )
    except asyncio.CancelledError:
        print("streaming was cancelled")
        return web.Response(text="Stream ended", status=200)
    except Exception as e:
        return web.Response(text=f"Error: {e}", status=500)


# Stream the latest frame to the client
async def stream_video_new(request):
    response = web.StreamResponse(
        status=200,
        reason='OK',
        headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-cache',
            'Connection': 'keep-alive'
        }
    )
    await response.prepare(request)

    while True:
        # Prepare a black canvas to display 4 streams (2x2 grid)
        canvas = np.zeros((960, 1280, 3), dtype=np.uint8)

        # Get frames from clients
        async with frame_lock:
            frames = list(request.app['video_frames'].items())

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

        await asyncio.sleep(1)  # Approx. 30 FPS


    while True:
        # Ensure latest_frame is available
        async with frame_lock:
            if latest_frame is None:
                await asyncio.sleep(0.1)  # Wait for the frame to arrive
                continue

        # Stream the latest frame
        await response.write(
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n\r\n'
        )
        await asyncio.sleep(1/30)  # Control the frame rate if necessary


def main():
    logging.basicConfig(level=logging.DEBUG)

    app = web.Application()
    app['video_frames'] = {}
    app['control_commands'] = []
    app.router.add_get('/', index)
    app.router.add_get('/video', stream_video)
    app.router.add_get('/ws', websocket_handler)
    # Serve static files (e.g., JS, CSS) from the 'static' directory
    app.router.add_static('/static', path='./static', name='static')

    web.run_app(app, host='0.0.0.0', port=8080)

if __name__ == "__main__":
    main()

