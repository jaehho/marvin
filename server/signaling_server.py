import asyncio
import json
import logging
import argparse
import websockets

# Setup logging.
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s"
)

# Dict mapping client IDs to (websocket, role).
connected_clients = {}

async def handler(websocket):
    client_id = None
    try:
        # Immediately expect a registration message.
        reg_message = await websocket.recv()
        data = json.loads(reg_message)
        if data.get("action") != "register" or "id" not in data or "role" not in data:
            await websocket.send(json.dumps({"error": "Invalid registration message"}))
            await websocket.close()
            return

        client_id = data["id"]
        role = data["role"]

        # Close previous connection if duplicate client_id.
        if client_id in connected_clients:
            logging.warning("Client '%s' already registered; closing previous connection.", client_id)
            old_ws, _ = connected_clients[client_id]
            await old_ws.close()

        connected_clients[client_id] = (websocket, role)
        logging.info("Registered client '%s' as '%s'.", client_id, role)

        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                await websocket.send(json.dumps({"error": "Invalid JSON format"}))
                continue

            # Heartbeat support.
            if data.get("action") == "ping":
                await websocket.send(json.dumps({"action": "pong"}))
                continue

            target = data.get("target")
            if not target:
                logging.warning("No target specified in message from '%s'.", client_id)
                continue

            # Broadcast support.
            if target == "broadcast":
                for cid, (ws, _) in connected_clients.items():
                    if cid == client_id:
                        continue
                    payload = data.copy()
                    payload["from"] = client_id
                    try:
                        await ws.send(json.dumps(payload))
                    except Exception as e:
                        logging.error("Error broadcasting to '%s': %s", cid, e)
            elif target in connected_clients:
                payload = data.copy()
                if "from" not in payload:
                    payload["from"] = client_id
                try:
                    target_ws, _ = connected_clients[target]
                    await target_ws.send(json.dumps(payload))
                except Exception as e:
                    logging.error("Error sending message from '%s' to '%s': %s", client_id, target, e)
            else:
                logging.warning("Unknown target '%s' from '%s'.", target, client_id)
                await websocket.send(json.dumps({"error": f"Unknown target: {target}"}))
    except websockets.exceptions.ConnectionClosed as e:
        logging.info("Connection closed for '%s': %s", client_id, e)
    except Exception as e:
        logging.error("Unexpected error with client '%s': %s", client_id, e)
    finally:
        if client_id and client_id in connected_clients:
            del connected_clients[client_id]
            logging.info("Removed client '%s' from registry.", client_id)

async def main(host: str, port: int):
    # Use a more forgiving ping configuration: every 30 seconds with a 60-second timeout.
    async with websockets.serve(handler, host, port, ping_interval=30, ping_timeout=60):
        logging.info("Signaling server running on ws://%s:%d", host, port)
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hub and Spoke Signaling Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host interface to bind")
    parser.add_argument("--port", type=int, default=8080, help="Port to listen on")
    args = parser.parse_args()
    asyncio.run(main(args.host, args.port))
