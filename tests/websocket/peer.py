# peer_ws.py
import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
import aiohttp

SIGNALING_URL = "ws://24.193.235.114:8080/ws"  # Replace with your signaling server IP
IS_OFFERER = input("Is this the offerer? (y/n): ").strip().lower() == "y"

TURN = RTCIceServer(
    urls="turn:24.193.235.114:3478",
    username="jaeho",
    credential="2121"
)
config = RTCConfiguration(iceServers=[TURN])

async def run():
    pc = RTCPeerConnection(configuration=config)

    @pc.on("iceconnectionstatechange")
    def on_state_change():
        print("ICE connection state:", pc.iceConnectionState)

    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        print("ICE candidate:", candidate)
        if candidate and ws is not None:
            msg = {
                "type": "candidate",
                "data": {
                    "candidate": candidate.sdp,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex
                }
            }
            await ws.send_str(json.dumps(msg))

    @pc.on("datachannel")
    def on_datachannel(channel):
        print("Data channel received")
        @channel.on("message")
        def on_message(message):
            print("Received:", message)
            channel.send("Hello back!")

    # Create channel if offerer
    if IS_OFFERER:
        channel = pc.createDataChannel("chat")

        @channel.on("open")
        def on_open():
            print("Data channel opened")
            channel.send("Hello from offerer!")

        @channel.on("message")
        def on_message(message):
            print("Received:", message)

    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(SIGNALING_URL) as socket:
            global ws
            ws = socket

            # Handle offer/answer flow
            if IS_OFFERER:
                offer = await pc.createOffer()
                await pc.setLocalDescription(offer)
                await ws.send_str(json.dumps({"type": "offer", "data": {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type
                }}))
            else:
                print("Waiting for offer...")

            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    message = json.loads(msg.data)

                    if message["type"] == "offer" and not IS_OFFERER:
                        offer = message["data"]
                        await pc.setRemoteDescription(RTCSessionDescription(**offer))
                        answer = await pc.createAnswer()
                        await pc.setLocalDescription(answer)
                        await ws.send_str(json.dumps({"type": "answer", "data": {
                            "sdp": pc.localDescription.sdp,
                            "type": pc.localDescription.type
                        }}))

                    elif message["type"] == "answer" and IS_OFFERER:
                        answer = message["data"]
                        await pc.setRemoteDescription(RTCSessionDescription(**answer))

                    elif message["type"] == "candidate":
                        c = message["data"]
                        candidate = RTCIceCandidate(
                            sdpMid=c["sdpMid"],
                            sdpMLineIndex=c["sdpMLineIndex"],
                            candidate=c["candidate"]
                        )
                        await pc.addIceCandidate(candidate)

                elif msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                    break

            await asyncio.sleep(60)

asyncio.run(run())
