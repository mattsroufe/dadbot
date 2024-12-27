import asyncio
import websockets
import numpy as np
import cv2
import io

# This will store the accumulated image data
image_data = bytearray()

async def handle_client(websocket):
    global image_data
    frame_number = 0
    print("Client connected")
    
    try:
        while True:
            # Receive binary data (image chunk)
            chunk = await websocket.recv()
            await websocket.send("capture " + str(frame_number))  # Send the "capture" message
            frame_number += 1
            # print(chunk);

            # Append the received chunk to the image data buffer
            image_data.extend(chunk)

            # Check if we have received enough data to form a complete image
            if len(image_data) > 0:
                # Try to decode the image if enough data is present
                nparr = np.frombuffer(image_data, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if image is not None:
                    # Show the image
                    cv2.imshow("Received Image", image)
                    
                    # Wait for 1 ms and check if the user presses the 'q' key to exit
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                    # Clear the image_data buffer after displaying the image
                    image_data = bytearray()

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    finally:
        cv2.destroyAllWindows()

async def main():
    # Start the WebSocket server
    server = await websockets.serve(handle_client, "0.0.0.0", 8080)
    print("WebSocket server started at ws://0.0.0.0:8080")
    
    # Keep the server running
    await server.wait_closed()

# Start the server
asyncio.run(main())

