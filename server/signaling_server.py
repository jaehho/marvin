# uvicorn signaling_server:app --host 0.0.0.0 --port 9000

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import json

app = FastAPI()
clients = {}

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        register = json.loads(await ws.receive_text())
        if register.get("type") != "register" or "id" not in register:
            await ws.close(1003)
            return
        client_id = register["id"]
        clients[client_id] = ws
        print(f"{client_id} connected to signaling server.")

        while True:
            raw = await ws.receive_text()
            msg = json.loads(raw)
            target = msg.get("to")
            print(f"{msg['from']} → {target}: {msg['type']}")
            if target in clients:
                await clients[target].send_text(raw)

    except WebSocketDisconnect:
        print(f"{client_id} disconnected.")
    finally:
        clients.pop(client_id, None)
