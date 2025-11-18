import asyncio
import websockets
import json

async def test():
    uri = "ws://localhost:8765"
    print(f"Connecting to {uri}...")
    async with websockets.connect(uri) as websocket:
        # Send "Fist"
        print("Sending: Fist")
        await websocket.send(json.dumps({"gesture": "Fist"}))
        await asyncio.sleep(1)

        # Send "Open_Palm"
        print("Sending: Open_Palm")
        await websocket.send(json.dumps({"gesture": "Open_Palm"}))

if __name__ == "__main__":
    asyncio.run(test())
