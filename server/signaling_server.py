import asyncio
import json
import websockets

connected = {}

async def handler(websocket):
    try:
        reg_message = await websocket.recv()
        data = json.loads(reg_message)
        if data.get("action") != "register" or "id" not in data or "role" not in data:
            await websocket.close()
            return
        client_id = data["id"]
        role = data["role"]
        print(f"Registered {client_id} as {role}")
        connected[client_id] = websocket

        async for message in websocket:
            data = json.loads(message)
            target = data.get("target")
            if target in connected:
                if "from" not in data:
                    data["from"] = client_id
                await connected[target].send(json.dumps(data))
            else:
                print(f"Unknown target: {target}")
    except websockets.exceptions.ConnectionClosed:
        print("Connection closed")
    finally:
        for key, ws in list(connected.items()):
            if ws == websocket:
                del connected[key]
                print(f"Removed {key}")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8080):
        print("Signaling server running on ws://0.0.0.0:8080")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
