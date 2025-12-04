import asyncio
import websockets
import json

async def test():
    uri = "ws://localhost:8765"
    print(f"Connecting to {uri}...")
    async with websockets.connect(uri) as websocket:
        
        print("Sending: forward (holding command for 5 seconds...)")

        # Send "forward" 10 times per second for 5 seconds
        for i in range(50):  # 50 * 0.1s = 5 seconds
            await websocket.send(json.dumps({"movement": "forward"}))
            await asyncio.sleep(0.1)

        print("Done. The robot should now coast until watchdog stops it.")

if __name__ == "__main__":
    asyncio.run(test())
