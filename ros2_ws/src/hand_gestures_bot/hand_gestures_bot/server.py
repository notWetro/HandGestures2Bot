import rclpy
import asyncio
import websockets
import json
from threading import Thread
from .turtlebot_controller import TurtleBotController
from .gesture_handler import GestureHandler

gesture_handler = None

async def websocket_listener(websocket, path):
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

def main(args=None):
    global gesture_handler
    rclpy.init(args=args)
    bot_node = TurtleBotController()
    gesture_handler = GestureHandler(bot_node)

    ros_thread = Thread(target=rclpy.spin, args=(bot_node,), daemon=True)
    ros_thread.start()

    print("WebSocket Server listening on port 8765...")
    start_server = websockets.serve(websocket_listener, "0.0.0.0", 8765)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(start_server)
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
