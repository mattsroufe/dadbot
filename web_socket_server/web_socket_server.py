import logging
import aiohttp
from aiohttp import web
import asyncio
import cv2
import numpy as np
import json
from concurrent.futures import ProcessPoolExecutor
from collections import deque
from time import time

WINDOW_NAME = "4 Streams Display"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 1/30

async def index(request):
    return web.Response(text=open('index.html').read(), content_type='text/html')

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    client_ip = request.remote

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            if msg.data == 'close':
                await ws.close()
            else:
                async with request.app['control_lock']:
                    request.app['control_commands'].clear()
                    request.app['control_commands'].extend(json.loads(msg.data))
        elif msg.type == aiohttp.WSMsgType.BINARY:
            frame_array = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
            async with request.app['frame_lock']:
                frame_queue = request.app['video_frames'].setdefault(client_ip, deque(maxlen=30))
                frame_queue.append((frame, time()))  # Store frame with timestamp

            try:
                ip_idx = list(request.app['video_frames']).index(client_ip)
                async with request.app['control_lock']:
                    command = request.app['control_commands'][ip_idx]
                await ws.send_str(f"CONTROL:{command[0]}:{command[1]}")
            except (IndexError, ValueError):
                pass
        elif msg.type == aiohttp.WSMsgType.ERROR:
            logging.error(f"WebSocket connection closed with exception {ws.exception()}")

    return ws


def process_frame_canvas(frame_queues):
    canvas = np.zeros((FRAME_HEIGHT * 2, FRAME_WIDTH * 2, 3), dtype=np.uint8)

    for i, (ip, frame_queue) in enumerate(frame_queues.items()):
        if not frame_queue:
            continue

        frame, _ = frame_queue[-1]  # Use the most recent frame
        resized_frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        x_offset = (i % 2) * FRAME_WIDTH
        y_offset = (i // 2) * FRAME_HEIGHT
        canvas[y_offset:y_offset+FRAME_HEIGHT, x_offset:x_offset+FRAME_WIDTH] = resized_frame

        # Calculate frame rate using timestamps
        timestamps = [ts for _, ts in frame_queue]
        frame_rate = None
        if len(timestamps) > 1:
            frame_rate = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])

        text_position_ip = (x_offset + 10, y_offset + 30)
        text_position_fps = (x_offset + 10, y_offset + 60)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (255, 255, 255)
        thickness = 2

        cv2.putText(canvas, ip, text_position_ip, font, font_scale, font_color, thickness)
        if frame_rate:
            cv2.putText(canvas, f"FPS: {frame_rate:.2f}", text_position_fps, font, font_scale, font_color, thickness)

    return canvas


async def generate_frames(request, pool):
    while True:
        async with request.app['frame_lock']:
            frame_queues = request.app['video_frames']
        if frame_queues:
            canvas = await asyncio.get_event_loop().run_in_executor(pool, process_frame_canvas, frame_queues)
        else:
            canvas = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

        _, jpeg_frame = cv2.imencode('.jpg', canvas)
        yield jpeg_frame.tobytes()

        await asyncio.sleep(FRAME_RATE)


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
        pool = request.app['process_pool']

        async for frame_data in generate_frames(request, pool):
            await response.write(
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n\r\n'
            )
    except asyncio.CancelledError:
        logging.info("Streaming was cancelled")
    except Exception as e:
        logging.error(f"Streaming error: {e}")
        return web.Response(text=f"Error: {e}", status=500)


def main():
    logging.basicConfig(level=logging.DEBUG)
    app = web.Application()
    app['video_frames'] = {}
    app['control_commands'] = []
    app['process_pool'] = ProcessPoolExecutor()
    app['frame_lock'] = asyncio.Lock()
    app['control_lock'] = asyncio.Lock()

    app.router.add_get('/', index)
    app.router.add_get('/video', stream_video)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_static('/static', path='./static', name='static')

    try:
        web.run_app(app, host='0.0.0.0', port=8080)
    finally:
        app['process_pool'].shutdown()


if __name__ == "__main__":
    main()
