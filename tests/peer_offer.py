# peer_a.py
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

    # Dummy data channel
    dc = pc.createDataChannel("chat")

    @dc.on("open")
    def on_open():
        print("Data channel is open")
        dc.send("Hello from Peer A!")

    @dc.on("message")
    def on_message(message):
        print("Received:", message)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    async with aiohttp.ClientSession() as session:
        async with session.post("http://localhost:8080/offer", json={
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }) as resp:
            answer = await resp.json()
            await pc.setRemoteDescription(RTCSessionDescription(**answer))

    await asyncio.sleep(60)

asyncio.run(run())
