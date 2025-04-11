import asyncio
import json
import logging
import argparse
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s"
)

# Map client IDs to their RTCPeerConnection instances.
peer_connections = {}

async def signaling_handler(signaling_uri: str):
    async with websockets.connect(signaling_uri, ping_interval=30, ping_timeout=60) as ws:
        # Register as host.
        registration = {
            "action": "register",
            "role": "host",
            "id": "host"
        }
        await ws.send(json.dumps(registration))
        logging.info("Registered as host (id: 'host').")

        async for message in ws:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                logging.error("Received invalid JSON message.")
                continue

            sender = data.get("from")
            msg_type = data.get("type")

            if msg_type == "offer":
                logging.info("Received offer from client '%s'.", sender)
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
                        logging.info("Sent ICE candidate to client '%s'.", client_id)

                @pc.on("datachannel")
                def on_datachannel(channel):
                    logging.info("Data channel established with client '%s': %s", sender, channel.label)
                    @channel.on("message")
                    def on_message(message):
                        logging.info("Received data channel message from '%s': %s", sender, message)

                try:
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
                    logging.info("Sent answer to client '%s'.", sender)
                except Exception as e:
                    logging.error("Error processing offer from '%s': %s", sender, e)

            elif msg_type == "candidate":
                if sender in peer_connections:
                    pc = peer_connections[sender]
                    candidate_data = data.get("candidate")
                    if candidate_data:
                        candidate = RTCIceCandidate(
                            candidate=candidate_data.get("candidate"),
                            sdpMid=candidate_data.get("sdpMid"),
                            sdpMLineIndex=candidate_data.get("sdpMLineIndex")
                        )
                        await pc.addIceCandidate(candidate)
                        logging.info("Added ICE candidate from client '%s'.", sender)
                    else:
                        logging.warning("Missing candidate data from client '%s'.", sender)
                else:
                    logging.warning("No peer connection found for client '%s'.", sender)
            else:
                logging.warning("Unknown message type '%s' from '%s'.", msg_type, sender)

async def main(signaling_uri: str):
    try:
        await signaling_handler(signaling_uri)
    except Exception as e:
        logging.error("Error in signaling handler: %s", e)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Peer Host for Hub and Spoke System")
    parser.add_argument("--signaling", type=str, default="ws://24.193.235.114:8080",
                        help="Signaling server URI (e.g. ws://<server>:8080)")
    args = parser.parse_args()
    asyncio.run(main(args.signaling))
