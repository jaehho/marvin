# app.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field, validator, ValidationError
import asyncio, contextlib, logging
from typing import List, Optional, Literal, Union
from datetime import datetime
import json, os

app = FastAPI(title="Pose Ingest API", version="1.1")

# ---- Schemas (compatible with MediaPipe Pose Landmarker) ----

class Landmark(BaseModel):
    x: float
    y: float
    z: float
    visibility: Optional[float] = None
    presence: Optional[float] = None

class WorldLandmark(BaseModel):
    x: float
    y: float
    z: float
    visibility: Optional[float] = None

LandmarkList = List[Landmark]
WorldLandmarkList = List[WorldLandmark]

class WSFrame(BaseModel):
    t_ms: int
    landmarks: LandmarkList
    world_landmarks: Optional[WorldLandmarkList] = None

    @validator("t_ms")
    def nonnegative(cls, v):  # noqa: N805
        if v < 0:
            raise ValueError("t_ms must be >= 0")
        return v

# ---- v2 extensions ----

class SourceDims(BaseModel):
    width: int
    height: int

class Quality(BaseModel):
    avg_visibility: Optional[float] = None
    avg_presence: Optional[float] = None

class PoseFrameV2(WSFrame):
    schema: Literal["pose.v2"]
    tracker: Literal["mediapipe-pose"]
    frame_id: int
    source: SourceDims
    quality: Optional[Quality] = None

    @validator("landmarks")
    def _validate_lm_33(cls, v):  # noqa: N805
        if len(v) != 33:
            raise ValueError("landmarks must have 33 items")
        return v

    @validator("world_landmarks")
    def _validate_wlm_33(cls, v):  # noqa: N805
        if v is None or len(v) != 33:
            raise ValueError("world_landmarks must have 33 items")
        return v

# Backward-compatible packet for legacy batch consumers
class PosePacket(BaseModel):
    tracker: Optional[Literal["mediapipe-pose"]] = "mediapipe-pose"
    frames: List[WSFrame]

# ---- Health ----
@app.get("/api/health")
def health():
    return {"ok": True}

# ---- Live stream over WebSocket ----

BATCH_SIZE = 30          # frames per write
BATCH_MS = 1000          # or every 1s, whichever first
MAX_QUEUE = 300          # backpressure
DATA_PATH = "/data/pose_ingest.jsonl"

RATE_THRESHOLD = 0.90    # explicit rate control signal
LEGACY_WARN_THRESHOLD = 0.80

FrameT = Union[WSFrame, PoseFrameV2]

# basic logger
logging.basicConfig(level=logging.INFO)

@app.websocket("/api/pose/v1/ws")
async def ws_pose(websocket: WebSocket):
    await websocket.accept()

    # advertise capability once per connection
    await websocket.send_json({"type": "server_caps", "schema": "pose.v2"})

    queue: asyncio.Queue[FrameT] = asyncio.Queue(MAX_QUEUE)
    stop = asyncio.Event()
    last_rate_sent_level = False

    async def writer():
        batch: List[FrameT] = []
        deadline = asyncio.get_event_loop().time() + (BATCH_MS/1000)
        os.makedirs(os.path.dirname(DATA_PATH), exist_ok=True)
        with open(DATA_PATH, "a", encoding="utf-8") as f:
            while not stop.is_set():
                timeout = max(0, deadline - asyncio.get_event_loop().time())
                try:
                    fr = await asyncio.wait_for(queue.get(), timeout=timeout)
                    batch.append(fr)
                    if len(batch) >= BATCH_SIZE:
                        deadline = asyncio.get_event_loop().time() + (BATCH_MS/1000)
                except asyncio.TimeoutError:
                    pass

                if batch and (len(batch) >= BATCH_SIZE or asyncio.get_event_loop().time() >= deadline):
                    legacy_frames: List[dict] = []
                    v2_frames: List[dict] = []
                    for b in batch:
                        base_dict = {"t_ms": b.t_ms, "landmarks": b.landmarks, "world_landmarks": getattr(b, "world_landmarks", None)}
                        legacy_frames.append(base_dict)
                        if isinstance(b, PoseFrameV2):
                            v2_frames.append(b.dict())

                    record = {
                        "received_at": datetime.utcnow().isoformat() + "Z",
                        "schema": "pose.v2" if v2_frames else "pose.v1",
                        "packet": {
                            "tracker": "mediapipe-pose",
                            "frames": legacy_frames,
                        },
                    }
                    if v2_frames:
                        record["packet_v2"] = {
                            "tracker": "mediapipe-pose",
                            "frames": v2_frames,
                        }

                    f.write(json.dumps(record, ensure_ascii=False) + "\n")
                    f.flush()
                    batch.clear()
                    deadline = asyncio.get_event_loop().time() + (BATCH_MS/1000)

    writer_task = asyncio.create_task(writer())

    try:
        ping_task = asyncio.create_task(_ping(websocket))
        while True:
            msg = await websocket.receive_json()
            try:
                frame: FrameT
                if msg.get("schema") == "pose.v2":
                    frame = PoseFrameV2(**msg)
                else:
                    frame = WSFrame(**msg)
            except ValidationError as e:
                await websocket.send_json({"error": "invalid_frame", "detail": str(e)})
                continue

            if not isinstance(frame, PoseFrameV2):
                if not frame.world_landmarks:
                    continue

            # representative sample log
            try:
                lm0 = frame.landmarks[0]
                logging.info(
                    f"[WS] schema={getattr(frame, 'schema', 'pose.v1')} "
                    f"t={frame.t_ms} "
                    f"lm0=({lm0.x:.3f},{lm0.y:.3f},{lm0.z:.3f})"
                )
            except Exception:
                pass

            with contextlib.suppress(asyncio.QueueFull):
                queue.put_nowait(frame)

            q = queue.qsize()
            if q > MAX_QUEUE * RATE_THRESHOLD and not last_rate_sent_level:
                await websocket.send_json({"type": "rate", "hz": 15})
                last_rate_sent_level = True
            elif q <= MAX_QUEUE * RATE_THRESHOLD and last_rate_sent_level:
                last_rate_sent_level = False

            if q > MAX_QUEUE * LEGACY_WARN_THRESHOLD:
                await websocket.send_json({"warning": "backpressure", "queued": q})

    except WebSocketDisconnect:
        pass
    finally:
        stop.set()
        await asyncio.gather(writer_task, return_exceptions=True)
        with contextlib.suppress(asyncio.CancelledError):
            ping_task.cancel()

async def _ping(ws: WebSocket):
    while True:
        await asyncio.sleep(20)
        try:
            await ws.send_json({"type": "ping", "t": datetime.utcnow().isoformat()+"Z"})
        except RuntimeError:
            return
