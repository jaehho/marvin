import asyncio, json, uuid
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
import aiohttp

SIGNALING_URL = "ws://24.193.235.114:8080/ws"
ROOM_ID = "my-room"
PEER_ID = str(uuid.uuid4())[:8]

async def run():
    pc = RTCPeerConnection()

    @pc.on("iceconnectionstatechange")
    def on_state_change():
        print("ICE:", pc.iceConnectionState)

    channel = pc.createDataChannel("chat")

    @channel.on("open")
    def on_open():
        print("Data channel opened")
        channel.send("Hello from peer " + PEER_ID)

    @channel.on("message")
    def on_msg(msg):
        print("Received from host:", msg)

    @pc.on("icecandidate")
    async def on_candidate(candidate):
        if candidate:
            await ws.send_json({
                "type": "candidate",
                "room": ROOM_ID,
                "peer_id": PEER_ID,
                "data": {
                    "candidate": candidate.sdp,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex
                }
            })

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(SIGNALING_URL) as ws:
            await ws.send_json({"action": "join", "room": ROOM_ID, "peer_id": PEER_ID})
            await ws.send_json({
                "type": "offer",
                "room": ROOM_ID,
                "peer_id": PEER_ID,
                "data": {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type
                }
            })

            async for msg in ws:
                data = json.loads(msg.data)

                if data["type"] == "answer":
                    await pc.setRemoteDescription(RTCSessionDescription(**data["data"]))

                elif data["type"] == "candidate-host":
                    c = data["data"]
                    await pc.addIceCandidate(RTCIceCandidate(
                        sdpMid=c["sdpMid"],
                        sdpMLineIndex=c["sdpMLineIndex"],
                        candidate=c["candidate"]
                    ))

asyncio.run(run())
