# signaling_server.py
import asyncio
from aiohttp import web

peers = {}

async def offer(request):
    data = await request.json()
    peers["offer"] = data
    while "answer" not in peers:
        await asyncio.sleep(0.1)
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
