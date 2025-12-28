import rclpy
import asyncio
import websockets
import netifaces
import json
import socket
from threading import Thread
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from .turtlebot_controller import TurtleBotController
from .gesture_handler import MovementHandler

movement_handler = None  # Global reference
connected_clients = set()  # Track connected WebSocket clients


def get_ip_address():
    for iface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(iface)
        if netifaces.AF_INET in addrs:
            for addr in addrs[netifaces.AF_INET]:
                ip = addr.get('addr')
                if ip and not ip.startswith("127."):
                    return ip
    return "127.0.0.1"


async def broadcast_message(message: str):
    """Broadcast a message to all connected WebSocket clients."""
    if connected_clients:
        await asyncio.gather(
            *[client.send(message) for client in connected_clients],
            return_exceptions=True
        )


async def websocket_handler(websocket):
    """Handle WebSocket connection - receive commands and send status updates."""
    global movement_handler
    
    # Register client
    connected_clients.add(websocket)
    print(f"Client connected. Total clients: {len(connected_clients)}")
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                current_movement = data.get("movement", None)

                if current_movement and movement_handler:
                    movement_handler.handle_movement(current_movement)
                else:
                    print(f"Invalid JSON or Handler not ready - Data: {data}")

            except Exception as e:
                print(f"WebSocket Error: {e}")
    finally:
        # Unregister client
        connected_clients.discard(websocket)
        print(f"Client disconnected. Total clients: {len(connected_clients)}")


async def start_websocket_server():
    """Start the WebSocket server."""
    ip_address = get_ip_address()
    print(f"WebSocket Server running on ws://{ip_address}:8765")
    async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever


class ObstacleStatusSubscriber:
    """Subscribes to obstacle status and broadcasts to WebSocket clients."""
    
    def __init__(self, node, loop):
        self.node = node
        self.loop = loop
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        self.subscription = node.create_subscription(
            String,
            '/obstacle_status',
            self.callback,
            qos
        )
        print("Subscribed to /obstacle_status for app feedback")
    
    def callback(self, msg):
        """Forward obstacle status to all connected clients."""
        if connected_clients:
            asyncio.run_coroutine_threadsafe(
                broadcast_message(msg.data),
                self.loop
            )


def main(args=None):
    global movement_handler
    rclpy.init(args=args)

    bot_node = TurtleBotController()
    movement_handler = MovementHandler(bot_node)

    # Get the event loop for async operations
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    # Create obstacle status subscriber
    obstacle_sub = ObstacleStatusSubscriber(bot_node, loop)

    ros_thread = Thread(target=rclpy.spin, args=(bot_node,), daemon=True)
    ros_thread.start()

    try:
        loop.run_until_complete(start_websocket_server())
    except KeyboardInterrupt:
        pass
    finally:
        bot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
