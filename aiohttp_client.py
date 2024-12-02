import asyncio
import cv2
import websockets
import signal

async def stream_video(video_index, camera_id, server_uri, stop_event):
    print("Initializing video capture...")
    cap = cv2.VideoCapture(video_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"Error: Unable to open video source {video_index}. Exiting.")
        return

    try:
        async with websockets.connect(server_uri) as websocket:
            print(f"Connected to WebSocket server at {server_uri}.")
            while not stop_event.is_set():
                # Capture a frame
                ret, frame = cap.read()
                if not ret:
                    print("Error: Unable to read frame. Retrying...")
                    await asyncio.sleep(1)  # Wait a bit before retrying
                    continue

                # Encode the frame as JPEG
                success, buffer = cv2.imencode('.jpg', frame)
                if not success:
                    print("Error: Frame encoding failed.")
                    break

                # Send the camera ID and encoded frame
                try:
                    await websocket.send(camera_id.encode('utf-8'))
                    await websocket.send(buffer.tobytes())
                except Exception as e:
                    print(f"Error sending data: {e}")
                    break

                # Simulate ~30 FPS
                await asyncio.sleep(1 / 30)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        cap.release()
        print("Video source released.")

async def main(video_index, camera_id, server_uri):
    stop_event = asyncio.Event()

    def handle_signal():
        print("\nSignal received. Shutting down...")
        stop_event.set()

    # Set up signal handlers
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGINT, handle_signal)
    loop.add_signal_handler(signal.SIGTERM, handle_signal)

    # Run the video streaming coroutine
    print("Starting video streaming...")
    await stream_video(video_index, camera_id, server_uri, stop_event)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Stream video to a WebSocket server.")
    parser.add_argument("--video_index", type=int, default=0, help="Index of the video source (default: 0)")
    parser.add_argument("--camera_id", type=str, required=True, help="Unique identifier for the camera")
    parser.add_argument("--server_uri", type=str, default="ws://localhost:8080", help="WebSocket server URI")
    args = parser.parse_args()

    try:
        asyncio.run(main(args.video_index, args.camera_id, args.server_uri))
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
