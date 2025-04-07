# peer_offer.py
import asyncio
import aiohttp
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceServer, RTCConfiguration

from _config import TURN, SIGNALING_URL

config = RTCConfiguration(iceServers=[TURN])

async def run():
    pc = RTCPeerConnection(configuration=config)

    @pc.on("iceconnectionstatechange")
    def on_state_change():
        print("ICE state:", pc.iceConnectionState)

    dc = pc.createDataChannel("chat")

    @dc.on("open")
    def on_open():
        print("Data channel opened")
        dc.send("Hello from Peer A")

    @dc.on("message")
    def on_message(message):
        print("Received:", message)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    async with aiohttp.ClientSession() as session:
        async with session.post(f"{SIGNALING_URL}/offer", json={
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }) as resp:
            answer = await resp.json()
            if "sdp" not in answer or "type" not in answer:
                raise Exception("Invalid answer received")

            await pc.setRemoteDescription(RTCSessionDescription(**answer))

    await asyncio.sleep(60)

asyncio.run(run())
