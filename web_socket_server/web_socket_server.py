import logging
import aiohttp
from aiohttp import web
import asyncio
import websockets
import cv2
import numpy as np
import json
from concurrent.futures import ProcessPoolExecutor

WINDOW_NAME = "4 Streams Display"

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
                request.app['video_frames'][client_ip] = frame

            try:
                ip_idx = list(request.app['video_frames']).index(client_ip)
                async with request.app['control_lock']:
                    command = request.app['control_commands'][ip_idx]
                await ws.send_str(f"CONTROL:{command[0]}:{command[1]}")
            except (IndexError, ValueError):
                # logging.warning(f"No command available for index {ip_idx}")
                pass
        elif msg.type == aiohttp.WSMsgType.ERROR:
            logging.error(f"WebSocket connection closed with exception {ws.exception()}")

    return ws

def process_frame_canvas(frames):
    canvas = np.zeros((960, 1280, 3), dtype=np.uint8)
    for i, (ip, frame) in enumerate(frames[:4]):
        resized_frame = cv2.resize(frame, (640, 480))
        x_offset = (i % 2) * 640
        y_offset = (i // 2) * 480
        canvas[y_offset:y_offset+480, x_offset:x_offset+640] = resized_frame
        cv2.putText(canvas, ip, (x_offset + 10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return canvas

async def generate_frames(request, pool):
    while True:
        async with request.app['frame_lock']:
            frames = list(request.app['video_frames'].items())
        if frames:
            canvas = await asyncio.get_event_loop().run_in_executor(pool, process_frame_canvas, frames)
            _, jpeg_frame = cv2.imencode('.jpg', canvas)
            yield jpeg_frame.tobytes()
        await asyncio.sleep(1/30)

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
