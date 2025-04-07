import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from aiortc.contrib.signaling import BYE

from ICE_SERVERS import ICE_SERVERS

async def run():
    pc = RTCPeerConnection(configuration=RTCConfiguration(ICE_SERVERS))

    channel = pc.createDataChannel("chat")

    @channel.on("open")
    def on_open():
        print("Data channel opened!")
        channel.send("Hello from Peer A")

    @channel.on("message")
    def on_message(message):
        print("Received message:", message)

    await pc.setLocalDescription(await pc.createOffer())
    print("Send this to Peer B:\n", pc.localDescription.sdp)

    # Wait for answer
    sdp = input("Paste answer SDP from Peer B:\n")
    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="answer"))

    # Keep the connection open
    await asyncio.sleep(60)

asyncio.run(run())
