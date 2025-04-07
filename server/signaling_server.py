# signaling_server.py
import asyncio
import time
from aiohttp import web

peers = {}

async def offer(request):
    data = await request.json()
    peers["offer"] = data
    print("Received offer")

    timeout = time.time() + 30  # Wait up to 30 seconds
    while "answer" not in peers:
        await asyncio.sleep(0.2)
        if time.time() > timeout:
            return web.json_response({"error": "Timeout waiting for answer"}, status=408)

    return web.json_response(peers["answer"])

async def answer(request):
    data = await request.json()
    if not data.get("sdp") or not data.get("type"):
        return web.Response(status=400, text="Invalid answer")
    peers["answer"] = data
    print("Received answer")
    return web.Response(text="ok")

app = web.Application()
app.add_routes([
    web.post("/offer", offer),
    web.post("/answer", answer)
])

web.run_app(app, host="0.0.0.0", port=8080)
