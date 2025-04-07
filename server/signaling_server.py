# signaling_server.py
import asyncio
from aiohttp import web
import time

peers = {}

async def offer(request):
    data = await request.json()
    peers["offer"] = data

    timeout = time.time() + 30  # wait up to 30 seconds
    while "answer" not in peers:
        await asyncio.sleep(0.2)
        if time.time() > timeout:
            return web.json_response({"error": "Timeout waiting for answer"}, status=408)

    return web.json_response(peers["answer"])

async def answer(request):
    data = await request.json()
    peers["answer"] = data
    return web.Response(text="ok")

app = web.Application()
app.add_routes([
    web.post("/offer", offer),
    web.post("/answer", answer)
])

web.run_app(app, port=8080)
