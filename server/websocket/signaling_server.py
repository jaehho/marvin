# signaling_ws_server.py
import asyncio
import json
from aiohttp import web, WSMsgType

clients = []
pending_offer = None

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    global pending_offer
    clients.append(ws)
    print("Client connected. Total:", len(clients))

    # Send stored offer if exists
    if pending_offer:
        await ws.send_str(pending_offer)

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                data = json.loads(msg.data)

                # If offer, store it
                if data.get("type") == "offer":
                    pending_offer = msg.data  # raw JSON string

                # Send to all others
                for client in clients:
                    if client != ws:
                        await client.send_str(msg.data)

    finally:
        clients.remove(ws)
        print("Client disconnected. Total:", len(clients))

    return ws

app = web.Application()
app.router.add_get("/ws", websocket_handler)

web.run_app(app, host="0.0.0.0", port=8080)
