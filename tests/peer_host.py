# host.py
import asyncio
import json
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate

# Mapping from client id to RTCPeerConnection instance.
peer_connections = {}

async def signaling_handler():
    uri = "ws://24.193.235.114:8080"
    async with websockets.connect(uri) as ws:
        # Register as host.
        await ws.send(json.dumps({
            "action": "register",
            "role": "host",
            "id": "host"
        }))
        print("Registered as host.")

        async for message in ws:
            data = json.loads(message)
            sender = data.get("from")
            msg_type = data.get("type")

            if msg_type == "offer":
                print(f"Received offer from client {sender}.")
                # Create a new RTCPeerConnection for this client.
                pc = RTCPeerConnection()
                peer_connections[sender] = pc

                @pc.on("icecandidate")
                async def on_icecandidate(event, client_id=sender):
                    candidate = event.candidate
                    if candidate:
                        candidate_message = json.dumps({
                            "type": "candidate",
                            "candidate": {
                                "candidate": candidate.candidate,
                                "sdpMid": candidate.sdpMid,
                                "sdpMLineIndex": candidate.sdpMLineIndex
                            },
                            "target": client_id,
                            "from": "host"
                        })
                        await ws.send(candidate_message)

                @pc.on("datachannel")
                def on_datachannel(channel):
                    print(f"Data channel from {sender}: {channel.label}")

                    @channel.on("message")
                    def on_message(message):
                        print(f"Received message from {sender}: {message}")
                        # (Optional) Process or broadcast the message.

                # Set remote description and create/send answer.
                offer = RTCSessionDescription(sdp=data["sdp"], type="offer")
                await pc.setRemoteDescription(offer)
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                answer_message = json.dumps({
                    "type": "answer",
                    "sdp": pc.localDescription.sdp,
                    "target": sender,
                    "from": "host"
                })
                await ws.send(answer_message)

            elif msg_type == "candidate":
                if sender in peer_connections:
                    pc = peer_connections[sender]
                    candidate_data = data["candidate"]
                    candidate = RTCIceCandidate(
                        candidate=candidate_data["candidate"],
                        sdpMid=candidate_data["sdpMid"],
                        sdpMLineIndex=candidate_data["sdpMLineIndex"]
                    )
                    await pc.addIceCandidate(candidate)
                else:
                    print(f"No peer connection found for client {sender}.")

async def main():
    await signaling_handler()

if __name__ == "__main__":
    asyncio.run(main())
