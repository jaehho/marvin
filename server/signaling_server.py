import json
from aiohttp import web, WSMsgType

rooms = {}

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    role = None
    room_id = None
    peer_id = None

    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            data = json.loads(msg.data)

            if data["action"] == "host":
                room_id = data["room"]
                rooms.setdefault(room_id, {"host": None, "peers": {}})
                rooms[room_id]["host"] = ws
                role = "host"
                print(f"Host joined room {room_id}")

            elif data["action"] == "join":
                room_id = data["room"]
                peer_id = data["peer_id"]
                rooms.setdefault(room_id, {"host": None, "peers": {}})
                rooms[room_id]["peers"][peer_id] = ws
                role = "peer"
                print(f"Peer {peer_id} joined room {room_id}")

            elif data["type"] in ("offer", "candidate"):
                # Peer sending to host
                to_host = rooms[data["room"]]["host"]
                await to_host.send_str(msg.data)

            elif data["type"] in ("answer", "candidate-host"):
                # Host sending to peer
                target_id = data["target"]
                target_ws = rooms[data["room"]]["peers"].get(target_id)
                if target_ws:
                    await target_ws.send_str(msg.data)

    # Cleanup
    if role == "peer" and peer_id and room_id:
        rooms[room_id]["peers"].pop(peer_id, None)
    elif role == "host" and room_id:
        rooms[room_id]["host"] = None

    return ws

app = web.Application()
app.router.add_get("/ws", websocket_handler)

web.run_app(app, host="0.0.0.0", port=8080)
