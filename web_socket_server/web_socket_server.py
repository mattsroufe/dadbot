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

LOCALHOST = '127.0.0.1'
WINDOW_NAME = "4 Streams Display"
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_RATE = 1/30

async def index(request):
    return web.Response(text=open('index.html').read(), content_type='text/html')

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    client_ip = request.remote

    async with request.app['client_lock']:
        request.app['websockets'][client_ip] = ws

    if client_ip is LOCALHOST:
        ws.send_json(list(request.app['video_frames'].keys()))

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            await handle_text_message(msg, request, ws)
        elif msg.type == aiohttp.WSMsgType.BINARY:
            await handle_binary_message(msg, client_ip, request, ws)
        elif msg.type == aiohttp.WSMsgType.ERROR:
            logging.error(f"WebSocket connection closed with exception {ws.exception()}")

    return ws

async def handle_text_message(msg, request, ws):
    if msg.data == 'close':
        await ws.close()
    else:
        request.app['control_commands'].clear()
        request.app['control_commands'].update(json.loads(msg.data))

async def handle_binary_message(msg, client_ip, request, ws):
    frame_queue = await get_or_create_frame_queue(client_ip, request, ws)
    frame_array = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
    frame_queue.append((frame, time()))  # Store frame with timestamp

    if client_ip in request.app['control_commands']:
        command = request.app['control_commands'][client_ip]
        await ws.send_str(f"CONTROL:{command[0]}:{command[1]}")

async def get_or_create_frame_queue(client_ip, request, ws):
    if client_ip not in request.app['video_frames']:
        async with request.app['frame_lock']:
            video_frames = request.app['video_frames']
            frame_queue = video_frames.setdefault(client_ip, deque(maxlen=10))
        if LOCALHOST in request.app['websockets']:
            await request.app['websockets'][LOCALHOST].send_str(json.dumps(list(video_frames.keys())))
    else:
        frame_queue = request.app['video_frames'].get(client_ip)
    return frame_queue

def process_frame_canvas(frame_queues):
    canvas = np.zeros((FRAME_HEIGHT * 2, FRAME_WIDTH * 2, 3), dtype=np.uint8)

    frame_rates = {}

    for i, (ip, frame_queue) in enumerate(frame_queues.items()):
        if not frame_queue:
            continue

        frame, _ = frame_queue[-1]  # Use the most recent frame
        # resized_frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        x_offset, y_offset = get_offsets(i)
        # canvas[y_offset:y_offset+FRAME_HEIGHT, x_offset:x_offset+FRAME_WIDTH] = resized_frame
        canvas[y_offset:y_offset+FRAME_HEIGHT, x_offset:x_offset+FRAME_WIDTH] = frame

        frame_rates[ip] = calculate_frame_rate(frame_queue)

        # Add text annotations
        # add_text_to_canvas(canvas, ip, x_offset, y_offset, frame_queue)

    logging.info(frame_rates)
    return canvas

def get_offsets(index):
    x_offset = (index % 2) * FRAME_WIDTH
    y_offset = (index // 2) * FRAME_HEIGHT
    return x_offset, y_offset

def add_text_to_canvas(canvas, ip, x_offset, y_offset, frame_queue):
    frame_rate = calculate_frame_rate(frame_queue)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    font_color = (255, 255, 255)
    thickness = 2

    cv2.putText(canvas, ip, (x_offset + 10, y_offset + 30), font, font_scale, font_color, thickness)
    if frame_rate:
        cv2.putText(canvas, f"FPS: {frame_rate:.2f}", (x_offset + 10, y_offset + 60), font, font_scale, font_color, thickness)

def calculate_frame_rate(frame_queue):
    timestamps = [ts for _, ts in frame_queue]
    frame_rate = 0
    number_of_frames = len(timestamps)
    if number_of_frames > 1:
        frame_rate = (number_of_frames - 1) / (timestamps[-1] - timestamps[0]) # minus one because time between frames not including frames
    return round(frame_rate, 1)

async def generate_frames(request, pool):
    shutdown_event = request.app['shutdown_event']
    while not shutdown_event.is_set():
        async with request.app['frame_lock']:
            frame_queues = request.app['video_frames']

        canvas = await asyncio.get_event_loop().run_in_executor(pool, process_frame_canvas, frame_queues)

        _, jpeg_frame = cv2.imencode('.jpg', canvas)
        yield jpeg_frame.tobytes()

        await asyncio.sleep(FRAME_RATE)

async def stream_video(request):
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
    await response.write_eof()
    return response

async def cleanup(app):
    logging.info("Shutting down process pool and stopping streams")
    app['shutdown_event'].set()  # Signal all streaming tasks to stop
    app['process_pool'].shutdown()

def main():
    logging.basicConfig(level=logging.DEBUG)
    app = web.Application()
    app['websockets'] = {}
    app['video_frames'] = {}
    app['control_commands'] = {}
    app['process_pool'] = ProcessPoolExecutor(max_workers=4)
    app['frame_lock'] = asyncio.Lock()
    app['client_lock'] = asyncio.Lock()
    app['shutdown_event'] = asyncio.Event()

    app.router.add_get('/', index)
    app.router.add_get('/video', stream_video)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_static('/static', path='./static', name='static')

    app.on_shutdown.append(cleanup)

    web.run_app(app, host='0.0.0.0', port=8080)

if __name__ == "__main__":
    main()

