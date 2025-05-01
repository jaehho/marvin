import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription
from websockets import connect

SIGNAL_URL = "ws://24.193.235.114:9000/ws"
HUB_ID = "hub"

# Track all active connections
clients = {}  # { client_id: { "pc": RTCPeerConnection, "dc": DataChannel } }

async def handle_offer(msg, ws):
    client_id = msg["from"]
    print(f"Offer received from {client_id}")
    
    offer = RTCSessionDescription(sdp=msg["sdp"], type=msg["type"])
    pc = RTCPeerConnection()
    clients[client_id] = {"pc": pc, "dc": None}

    @pc.on("datachannel")
    def on_datachannel(channel):
        print(f"DataChannel opened from {client_id}")
        clients[client_id]["dc"] = channel

        @channel.on("message")
        def on_message(message):
            print(f"From {client_id}: {message}")
            # # Broadcast to all other clients
            # for other_id, info in clients.items():
            #     if other_id != client_id and info["dc"]:
            #         info["dc"].send(f"[{client_id}] {message}")
            #         print(f"Relayed to {other_id}")
            # Only send the message back to the originating client (if required)
            if client_id in clients:
                info = clients.get(client_id)
                if info and info["dc"]:
                    info["dc"].send(f"[hub] {message}")  # Forward the message back to the client
                    print(f"Relayed message to {client_id}")


    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    # Send answer back
    await ws.send(json.dumps({
        "type": pc.localDescription.type,
        "sdp": pc.localDescription.sdp,
        "from": HUB_ID,
        "to": client_id
    }))
    print(f"Sent answer to {client_id}")

async def handle_candidate(msg):
    client_id = msg["from"]
    pc = clients.get(client_id, {}).get("pc")
    if not pc:
        print(f"No RTCPeerConnection found for {client_id}")
        return

    cand = msg["candidate"]
    try:
        await pc.addIceCandidate(cand)
        print(f"ICE candidate added for {client_id}")
    except Exception as e:
        # print(f"Failed to add ICE candidate for {client_id}: {e}")
        print(".", end="")

async def hub_main():
    async with connect(SIGNAL_URL) as ws:
        # Register this client (the hub)
        await ws.send(json.dumps({"type": "register", "id": HUB_ID}))
        print("Hub connected to signaling server")

        async for msg in ws:
            try:
                data = json.loads(msg)
                if data["type"] == "offer":
                    await handle_offer(data, ws)
                elif data["type"] == "candidate":
                    await handle_candidate(data)
                else:
                    print(f"Unknown message type: {data['type']}")
            except Exception as e:
                print(f"Error processing message: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(hub_main())
    except KeyboardInterrupt:
        print("Hub shutdown requested")
