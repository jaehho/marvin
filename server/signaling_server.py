# signaling_server.py (final version)
import asyncio
import time
from aiohttp import web

peers = {}

async def handle_offer(request):
    if request.method == 'POST':
        data = await request.json()
        peers["offer"] = data
        print("Received offer")
        return web.Response(text="OK")

    elif request.method == 'GET':
        timeout = time.time() + 30
        while "offer" not in peers:
            await asyncio.sleep(0.2)
            if time.time() > timeout:
                return web.json_response({"error": "Timeout waiting for offer"}, status=408)
        return web.json_response(peers["offer"])

async def handle_answer(request):
    data = await request.json()
    if not data.get("sdp") or not data.get("type"):
        return web.Response(status=400, text="Invalid answer")
    peers["answer"] = data
    print("Received answer")
    return web.json_response(data)

async def get_answer(request):
    timeout = time.time() + 30
    while "answer" not in peers:
        await asyncio.sleep(0.2)
        if time.time() > timeout:
            return web.json_response({"error": "Timeout waiting for answer"}, status=408)
    return web.json_response(peers["answer"])

app = web.Application()
app.add_routes([
    web.route("*", "/offer", handle_offer),
    web.post("/answer", handle_answer),
    web.get("/get-answer", get_answer)
])

web.run_app(app, host="0.0.0.0", port=8080)
