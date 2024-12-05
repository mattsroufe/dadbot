import asyncio
import websockets
import cv2
import numpy as np

# Dictionary to store frames from ESP32-CAM
frames = {}

async def handle_client(websocket):
    async for message in websocket:
        if message.startswith(b"CAM"): # Metadata
            cam_id = message[:4].decode('utf-8')
            frames[cam_id] = cv2.imdecode(np.frombuffer(message[4:], np.uint8), cv2.IMREAD_COLOR)

async def display_frames():
    while True:
        if frames:
            display = np.zeros((480 * 2, 640 * 2, 3), dtype=np.uint8)
            idx = 0
            for cam_id, frame in frames.items():
                x = (idx % 2) * 640
                y = (idx // 2) * 480
                display[y:y+480, x:x+640] = cv2.resize(frame, (640, 480))
                idx += 1
                cv2.imshow('Cameras', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        await asyncio.sleep(0.1)

async def main():
    server = await websockets.serve(handle_client, "0.0.0.0", 8080,
                                    ping_interval=20, ping_timeout=10)
    await asyncio.gather(server.wait_closed(), display_frames())

asyncio.run(main())
