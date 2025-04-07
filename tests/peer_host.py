import asyncio, json, uuid
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
import aiohttp

SIGNALING_URL = "ws://24.193.235.114:8080/ws"
ROOM_ID = "my-room"

pcs = {}

async def run():
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(SIGNALING_URL) as ws:
            await ws.send_json({"action": "host", "room": ROOM_ID})
            print("Host connected to signaling server.")

            async for msg in ws:
                data = json.loads(msg.data)

                if data["type"] == "offer":
                    peer_id = data["peer_id"]
                    pc = RTCPeerConnection()
                    pcs[peer_id] = pc

                    @pc.on("icecandidate")
                    async def on_candidate(candidate):
                        if candidate:
                            await ws.send_json({
                                "type": "candidate-host",
                                "room": ROOM_ID,
                                "target": peer_id,
                                "data": {
                                    "candidate": candidate.sdp,
                                    "sdpMid": candidate.sdpMid,
                                    "sdpMLineIndex": candidate.sdpMLineIndex
                                }
                            })

                    @pc.on("datachannel")
                    def on_datachannel(channel):
                        @channel.on("message")
                        def on_message(msg):
                            print(f"[{peer_id}] {msg}")
                            channel.send(f"Hello {peer_id} from host!")

                    offer = data["data"]
                    await pc.setRemoteDescription(RTCSessionDescription(**offer))
                    answer = await pc.createAnswer()
                    await pc.setLocalDescription(answer)

                    await ws.send_json({
                        "type": "answer",
                        "room": ROOM_ID,
                        "target": peer_id,
                        "data": {
                            "sdp": pc.localDescription.sdp,
                            "type": pc.localDescription.type
                        }
                    })

                elif data["type"] == "candidate":
                    peer_id = data["peer_id"]
                    candidate = data["data"]
                    await pcs[peer_id].addIceCandidate(RTCIceCandidate(
                        sdpMid=candidate["sdpMid"],
                        sdpMLineIndex=candidate["sdpMLineIndex"],
                        candidate=candidate["candidate"]
                    ))

asyncio.run(run())
