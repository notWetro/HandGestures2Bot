import asyncio
import websockets
import json

async def test():
    uri = "ws://localhost:8765"
    print(f"Connecting to {uri}...")
    async with websockets.connect(uri) as websocket:
        
        print("Sending: Fist (Holding gesture for 5 seconds...)")
        
        # Send "Fist" 10 times a second for 5 seconds
        for i in range(50): 
            await websocket.send(json.dumps({"gesture": "Fist"}))
            await asyncio.sleep(0.1) # Wait 0.1s between messages

        print("Finished. Watchdog should stop it now.")

if __name__ == "__main__":
    asyncio.run(test())