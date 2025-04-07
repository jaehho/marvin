# peer_b.py
import asyncio
import json
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
import aiohttp

from _config import TURN

async def run():
    config = RTCConfiguration(iceServers=[TURN])
    pc = RTCPeerConnection(configuration=config)

    @pc.on("iceconnectionstatechange")
    def on_state_change():
        print("Connection state:", pc.iceConnectionState)

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            print("Received:", message)
            channel.send("Hello from Peer B!")

    async with aiohttp.ClientSession() as session:
        while True:
            async with session.post("http://localhost:8080/answer", json={
                "sdp": "",
                "type": ""
            }) as resp:
                if resp.status == 200:
                    break
            await asyncio.sleep(0.5)

        async with session.get("http://localhost:8080/offer") as resp:
            offer = await resp.json()

        await pc.setRemoteDescription(RTCSessionDescription(**offer))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        async with session.post("http://localhost:8080/answer", json={
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }) as resp:
            print("Sent answer")

    await asyncio.sleep(60)

asyncio.run(run())
