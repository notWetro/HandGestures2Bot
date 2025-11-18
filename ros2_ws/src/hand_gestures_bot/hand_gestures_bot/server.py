import rclpy
import asyncio
import websockets
import json
from threading import Thread
from .turtlebot_controller import TurtleBotController
from .gesture_handler import GestureHandler

# Global reference
gesture_handler = None

async def websocket_listener(websocket):
    global gesture_handler
    async for message in websocket:
        try:
            data = json.loads(message)
            current_gesture = data.get("gesture", None)
            
            if current_gesture and gesture_handler:
                gesture_handler.handle_gesture(current_gesture)
            else:
                print("Invalid JSON or Handler not ready")

        except Exception as e:
            print(f"WebSocket Error: {e}")

async def start_websocket_server():
    # Modern AsyncIO starter
    print("WebSocket Server listening on port 8765...")
    async with websockets.serve(websocket_listener, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

def main(args=None):
    global gesture_handler
    rclpy.init(args=args)
    
    # Initialize ROS Node
    bot_node = TurtleBotController()
    gesture_handler = GestureHandler(bot_node)

    # Run ROS in a separate thread so it doesn't block the WebSocket
    ros_thread = Thread(target=rclpy.spin, args=(bot_node,), daemon=True)
    ros_thread.start()

    # Run the WebSocket Server using the modern asyncio.run()
    try:
        asyncio.run(start_websocket_server())
    except KeyboardInterrupt:
        pass
    finally:
        bot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()