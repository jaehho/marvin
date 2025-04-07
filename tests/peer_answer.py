# peer_answer.py
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

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            print("Received:", message)
            channel.send("Hello from Peer B")

    async with aiohttp.ClientSession() as session:
        async with session.get(f"{SIGNALING_URL}/offer") as resp:
            if resp.status != 200:
                raise Exception("No offer available")
            offer = await resp.json()

        await pc.setRemoteDescription(RTCSessionDescription(**offer))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        await session.post(f"{SIGNALING_URL}/answer", json={
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })

    await asyncio.sleep(60)

asyncio.run(run())
