import rclpy
import asyncio
import websockets
import json
from threading import Thread
from .turtlebot_controller import TurtleBotController
from .gesture_handler_handler import MovementHandler

movement_handler = None  # Global reference

async def websocket_listener(websocket):
    global movement_handler
    async for message in websocket:
        try:
            data = json.loads(message)
            current_movement = data.get("movement", None)

            if current_movement and movement_handler:
                movement_handler.handle_movement(current_movement)
            else:
                print("Invalid JSON or Handler not ready")

        except Exception as e:
            print(f"WebSocket Error: {e}")

async def start_websocket_server():
    print("WebSocket Server running on ws://0.0.0.0:8765")
    async with websockets.serve(websocket_listener, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

def main(args=None):
    global movement_handler
    rclpy.init(args=args)

    bot_node = TurtleBotController()
    movement_handler = MovementHandler(bot_node)

    ros_thread = Thread(target=rclpy.spin, args=(bot_node,), daemon=True)
    ros_thread.start()

    try:
        asyncio.run(start_websocket_server())
    except KeyboardInterrupt:
        pass
    finally:
        bot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
