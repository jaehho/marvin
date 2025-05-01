# uvicorn signaling_server:app --host 0.0.0.0 --port 9000

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import json

app = FastAPI()
clients = {}  # Maps client ids to WebSocket objects
queue = []  # A list to track the clients in the queue
is_busy = False  # A flag to track if the system is busy
current_control = None  # Track which client currently has control

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    global is_busy, current_control
    await ws.accept()
    try:
        register = json.loads(await ws.receive_text())
        if register.get("type") != "register" or "id" not in register:
            await ws.close(1003)
            return
        client_id = register["id"]
        clients[client_id] = ws
        print(f"[DEBUG] {client_id} connected to signaling server.")
        
        # Send the current control, busy status, and the ordered queue to the newly connected client
        queue_ordered = list(queue)  # Get a fresh ordered list of clients in the queue
        await ws.send_text(json.dumps({
            "type": "status",
            "status": "busy" if is_busy and current_control != client_id else "available",
            "control": current_control,
            "queue": queue_ordered,  # Send the ordered list of clients in the queue
        }))
        print(f"[DEBUG] Sending current status to {client_id}: control={current_control}, queue={queue_ordered}")

        while True:
            raw = await ws.receive_text()
            msg = json.loads(raw)
            target = msg.get("to")
            print(f"[DEBUG] {msg['from']} → {target}: {msg['type']}")

            if msg["type"] == "message":
                if is_busy and current_control != client_id:
                    await ws.send_text(json.dumps({
                        "type": "busy",
                        "message": "System is busy, please try again later."
                    }))
                else:
                    is_busy = True
                    if target in clients:
                        await clients[target].send_text(raw)
                    is_busy = False

            elif msg["type"] == "claim_control":
                if current_control is None and len(queue) == 0:
                    current_control = client_id
                    is_busy = True
                    print(f"[DEBUG] {client_id} claimed control.")
                    # Inform all clients about the new control status
                    for client in clients.values():
                        await client.send_text(json.dumps({
                            "type": "status",
                            "status": "busy",
                            "control": current_control,
                            "queue": queue,  # Send the ordered queue
                        }))
                else:
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": "Control is already claimed or there are clients in the queue."
                    }))
                    print(f"[DEBUG] {client_id} could not claim control.")

            elif msg["type"] == "give_up_control":
                if current_control == client_id:
                    current_control = None
                    is_busy = False
                    print(f"[DEBUG] {client_id} gave up control.")
                    for client in clients.values():
                        await client.send_text(json.dumps({
                            "type": "status",
                            "status": "available",
                            "control": current_control,
                            "queue": queue,
                        }))
                    if queue:
                        next_client = queue.pop(0)
                        current_control = next_client
                        is_busy = True
                        print(f"[DEBUG] {next_client} moved to control.")
                        for client in clients.values():
                            await client.send_text(json.dumps({
                                "type": "status",
                                "status": "busy",
                                "control": current_control,
                                "queue": queue,
                            }))
                else:
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": "You don't have control to give up."
                    }))

            elif msg["type"] == "join_queue":
                if client_id not in queue and current_control != client_id:
                    queue.append(client_id)
                    print(f"[DEBUG] {client_id} joined the queue. Queue length: {len(queue)}")
                    
                    # Send the updated ordered queue and each client's position in the queue
                    for client in clients.values():
                        queue_with_positions = [
                            {"client_id": client_id, "queue_position": idx + 1}
                            for idx, client_id in enumerate(queue)
                        ]
                        await client.send_text(json.dumps({
                            "type": "queue_update",
                            "queue": queue_with_positions,  # Send the updated ordered queue with positions
                        }))
                    
                    await ws.send_text(json.dumps({
                        "type": "queue_status",
                        "queue": queue,  # Send the updated ordered queue to the client
                    }))
                else:
                    await ws.send_text(json.dumps({
                        "type": "error",
                        "message": "You cannot join the queue if you already have control or are in the queue."
                    }))
                    print(f"[DEBUG] {client_id} could not join the queue.")
                    
            elif msg["type"] == "leave_queue":
                if client_id in queue:
                    queue.remove(client_id)
                    print(f"[DEBUG] {client_id} left the queue. Queue length: {len(queue)}")
                    
                    # Recalculate positions after removal
                    for client in clients.values():
                        queue_with_positions = [
                            {"client_id": client_id, "queue_position": idx + 1}
                            for idx, client_id in enumerate(queue)
                        ]
                        await client.send_text(json.dumps({
                            "type": "queue_update",
                            "queue": queue_with_positions,  # Send the updated ordered queue with positions
                        }))
                    
                    await ws.send_text(json.dumps({
                        "type": "queue_status",
                        "queue": queue,  # Send the updated ordered queue to the client
                    }))

    except WebSocketDisconnect:
        print(f"[DEBUG] {client_id} disconnected.")
        clients.pop(client_id, None)
        if current_control == client_id:
            current_control = None
            is_busy = False
            print(f"[DEBUG] {client_id} lost control and disconnected.")
            for client in clients.values():
                await client.send_text(json.dumps({
                    "type": "status",
                    "status": "available",
                    "control": current_control,
                    "queue": queue,
                }))
        if client_id in queue:
            queue.remove(client_id)
            print(f"[DEBUG] {client_id} was removed from the queue.")
            for client in clients.values():
                await client.send_text(json.dumps({
                    "type": "queue_update",
                    "queue": queue,  # Send the updated ordered queue
                }))
