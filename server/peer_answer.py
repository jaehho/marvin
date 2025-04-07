# peer_b.py
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer

from ICE_SERVERS import ICE_SERVERS

async def run():
    pc = RTCPeerConnection(configuration=RTCConfiguration(ICE_SERVERS))

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("open")
        def on_open():
            print("Data channel opened!")
            channel.send("Hello from Peer B")

        @channel.on("message")
        def on_message(message):
            print("Received message:", message)

    # Get offer from Peer A
    sdp = input("Paste offer SDP from Peer A:\n")
    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))

    await pc.setLocalDescription(await pc.createAnswer())
    print("Send this to Peer A:\n", pc.localDescription.sdp)

    # Keep the connection open
    await asyncio.sleep(60)

asyncio.run(run())
