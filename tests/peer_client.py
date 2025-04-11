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

async def send_channel_pings(channel, interval=5):
    """Periodically send a ping over the data channel to keep ICE consent alive."""
    while True:
        try:
            channel.send("ping")
            logging.info("Sent channel ping")
            await asyncio.sleep(interval)
        except Exception as e:
            logging.error("Error sending channel ping: %s", e)
            break

async def run_client(signaling_uri: str, client_id: str):
    async with websockets.connect(signaling_uri, ping_interval=30, ping_timeout=60) as ws:
        # Register as a client.
        registration = {
            "action": "register",
            "role": "client",
            "id": client_id
        }
        await ws.send(json.dumps(registration))
        logging.info("Registered as client (id: '%s').", client_id)

        pc = RTCPeerConnection()

        # Create a data channel.
        channel = pc.createDataChannel("chat")

        @channel.on("open")
        def on_open():
            logging.info("Data channel is open!")
            channel.send("Hello from client!")
            # Start sending pings every 5 seconds.
            asyncio.create_task(send_channel_pings(channel, interval=5))

        @channel.on("message")
        def on_message(message):
            logging.info("Received message from host: %s", message)

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
                logging.info("Sent ICE candidate to host.")

        try:
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
            logging.info("Sent offer to host.")
        except Exception as e:
            logging.error("Error creating/sending offer: %s", e)

        async for message in ws:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                logging.error("Received invalid JSON message.")
                continue

            msg_type = data.get("type")
            if msg_type == "answer":
                logging.info("Received answer from host.")
                try:
                    answer = RTCSessionDescription(sdp=data["sdp"], type="answer")
                    await pc.setRemoteDescription(answer)
                except Exception as e:
                    logging.error("Error setting remote description: %s", e)
            elif msg_type == "candidate":
                candidate_data = data.get("candidate")
                if candidate_data:
                    candidate = RTCIceCandidate(
                        candidate=candidate_data.get("candidate"),
                        sdpMid=candidate_data.get("sdpMid"),
                        sdpMLineIndex=candidate_data.get("sdpMLineIndex")
                    )
                    await pc.addIceCandidate(candidate)
                    logging.info("Added ICE candidate from host.")
                else:
                    logging.warning("Candidate data missing in candidate message.")
            else:
                logging.warning("Unknown message type received: %s", msg_type)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Peer Client for Hub and Spoke System")
    parser.add_argument("--signaling", type=str, default="ws://24.193.235.114:8080",
                        help="Signaling server URI (e.g. ws://<server>:8080)")
    parser.add_argument("--id", type=str, default="client1", help="Unique client ID")
    args = parser.parse_args()

    try:
        asyncio.run(run_client(args.signaling, args.id))
    except KeyboardInterrupt:
        logging.info("Client shutdown requested.")
    except Exception as e:
        logging.error("Error running client: %s", e)
