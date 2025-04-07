# client.py
import asyncio
import json
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate

async def run_client():
    uri = "ws://24.193.235.114:8080"
    async with websockets.connect(uri) as ws:
        # Use a unique id for the client.
        client_id = "client1"
        await ws.send(json.dumps({
            "action": "register",
            "role": "client",
            "id": client_id
        }))
        print(f"Registered as client with id {client_id}.")

        pc = RTCPeerConnection()

        # Create a data channel for bidirectional communication.
        channel = pc.createDataChannel("chat")

        @channel.on("open")
        def on_open():
            print("Data channel is open!")
            channel.send("Hello from client!")

        @channel.on("message")
        def on_message(message):
            print("Received message from host:", message)

        @pc.on("icecandidate")
        async def on_icecandidate(event):
            candidate = event.candidate
            if candidate:
                candidate_message = json.dumps({
                    "type": "candidate",
                    "candidate": {
                        "candidate": candidate.candidate,
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex
                    },
                    "target": "host",
                    "from": client_id
                })
                await ws.send(candidate_message)

        # Create and send the SDP offer.
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        offer_message = json.dumps({
            "type": "offer",
            "sdp": pc.localDescription.sdp,
            "target": "host",
            "from": client_id
        })
        await ws.send(offer_message)

        # Listen for messages (answer and ICE candidates) from the signaling server.
        async for message in ws:
            data = json.loads(message)
            msg_type = data.get("type")
            if msg_type == "answer":
                print("Received answer from host.")
                answer = RTCSessionDescription(sdp=data["sdp"], type="answer")
                await pc.setRemoteDescription(answer)
            elif msg_type == "candidate":
                candidate_data = data["candidate"]
                candidate = RTCIceCandidate(
                    candidate=candidate_data["candidate"],
                    sdpMid=candidate_data["sdpMid"],
                    sdpMLineIndex=candidate_data["sdpMLineIndex"]
                )
                await pc.addIceCandidate(candidate)

if __name__ == "__main__":
    asyncio.run(run_client())
