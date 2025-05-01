# Run with: uvicorn signaling_server:app --host 0.0.0.0 --port 9000

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import json

app = FastAPI()
clients = {}  # Maps client_id to WebSocket
queue = []  # Ordered list of client IDs in the queue
is_busy = False
current_control = None

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    global is_busy, current_control
    await ws.accept()
    client_id = None

    try:
        register = json.loads(await ws.receive_text())
        if register.get("type") != "register" or "id" not in register:
            await ws.close(1003)
            return

        client_id = register["id"]
        clients[client_id] = ws
        print(f"[DEBUG] {client_id} connected.")

        # Initial status update to client
        await ws.send_text(json.dumps({
            "type": "status",
            "status": "busy" if is_busy and current_control != client_id else "available",
            "control": current_control,
            "queue": queue
        }))
        print(f"[DEBUG] Sent initial status to {client_id}")

        while True:
            raw = await ws.receive_text()
            msg = json.loads(raw)
            msg_type = msg.get("type")
            target = msg.get("to")
            sender = msg.get("from")

            print(f"[DEBUG] {sender} → {target}: {msg_type}")

            if msg_type == "message":
                if is_busy and current_control != sender:
                    await ws.send_text(json.dumps({
                        "type": "busy",
                        "message": "System is busy, please try again later."
                    }))
                elif target in clients:
                    await clients[target].send_text(raw)

            elif msg_type == "claim_control":
                if current_control is None and len(queue) == 0:
                    current_control = sender
                    is_busy = True
                    print(f"[DEBUG] {sender} claimed control.")
                    await broadcast_status()
                else:
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": "Control is already claimed or queue is not empty."
                    }))

            elif msg_type == "give_up_control":
                if current_control == sender:
                    current_control = None
                    is_busy = False
                    print(f"[DEBUG] {sender} gave up control.")
                    if queue:
                        next_client = queue.pop(0)
                        current_control = next_client
                        is_busy = True
                        print(f"[DEBUG] {next_client} now has control.")
                    await broadcast_status()
                    await broadcast_queue()
                else:
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": "You don't have control."
                    }))

            elif msg_type == "join_queue":
                if sender not in queue and current_control != sender:
                    queue.append(sender)
                    print(f"[DEBUG] {sender} joined the queue.")
                    await broadcast_queue()
                    await ws.send_text(json.dumps({
                        "type": "queue_status",
                        "queue": queue
                    }))
                else:
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": "Already in queue or you have control."
                    }))

            elif msg_type == "leave_queue":
                if sender in queue:
                    queue.remove(sender)
                    print(f"[DEBUG] {sender} left the queue.")
                    await broadcast_queue()
                    await ws.send_text(json.dumps({
                        "type": "queue_status",
                        "queue": queue
                    }))

            elif msg_type in ("offer", "answer", "candidate"):
                if target in clients:
                    await clients[target].send_text(raw)
                else:
                    print(f"[WARN] Target {target} not connected.")
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": f"Target {target} not found."
                    }))

            else:
                print(f"[WARN] Unknown message type: {msg_type}")
                await ws.send_text(json.dumps({
                    "type": "error",
                    "message": f"Unknown message type: {msg_type}"
                }))

    except WebSocketDisconnect:
        print(f"[DEBUG] {client_id} disconnected.")
        clients.pop(client_id, None)
        if current_control == client_id:
            current_control = None
            is_busy = False
            print(f"[DEBUG] {client_id} was in control and disconnected.")
            if queue:
                current_control = queue.pop(0)
                is_busy = True
                print(f"[DEBUG] {current_control} now has control.")
        if client_id in queue:
            queue.remove(client_id)
            print(f"[DEBUG] {client_id} removed from queue.")
        await broadcast_status()
        await broadcast_queue()


async def broadcast_status():
    for ws in clients.values():
        await ws.send_text(json.dumps({
            "type": "status",
            "status": "busy" if is_busy else "available",
            "control": current_control,
            "queue": queue
        }))

async def broadcast_queue():
    queue_with_positions = [
        {"client_id": cid, "queue_position": idx + 1}
        for idx, cid in enumerate(queue)
    ]
    for ws in clients.values():
        await ws.send_text(json.dumps({
            "type": "queue_update",
            "queue": queue_with_positions
        }))
