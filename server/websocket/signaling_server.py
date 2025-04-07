# signaling_ws_server.py
import asyncio
import json
from aiohttp import web, WSMsgType

clients = []

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    clients.append(ws)
    print("Client connected. Total:", len(clients))

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                for client in clients:
                    if client != ws:
                        await client.send_str(msg.data)
            elif msg.type == WSMsgType.ERROR:
                print("WS connection closed with exception:", ws.exception())
    finally:
        clients.remove(ws)
        print("Client disconnected. Total:", len(clients))

    return ws

app = web.Application()
app.router.add_get("/ws", websocket_handler)

web.run_app(app, host="0.0.0.0", port=8080)
