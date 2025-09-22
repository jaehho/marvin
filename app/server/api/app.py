from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, ValidationError, field_validator
from typing import List, Optional
from datetime import datetime, timezone
import asyncio, contextlib, json, logging, os, errno

app = FastAPI(title="Pose Ingest API", version="2.0")

# ---------- Standard schema: accept any pose.* ----------
class Landmark(BaseModel):
    x: float; y: float; z: float
    visibility: Optional[float] = None
    presence: Optional[float] = None

class WorldLandmark(BaseModel):
    x: float; y: float; z: float
    visibility: Optional[float] = None

class SourceDims(BaseModel):
    width: int; height: int

class Quality(BaseModel):
    avg_visibility: Optional[float] = None
    avg_presence: Optional[float] = None

class PoseFrameV2(BaseModel):
    schema: str
    tracker: str
    frame_id: int
    t_ms: int
    source: SourceDims
    landmarks: List[Landmark]
    world_landmarks: List[WorldLandmark]
    quality: Optional[Quality] = None

    # 1) relax schema check: allow any "pose.*" or anything truthy; only warn at runtime if needed
    @field_validator("schema")
    @classmethod
    def _schema_relaxed(cls, v: str):
        if not v:
            raise ValueError("schema is required")
        return v

    # keep tracker check as-is (was not requested to relax)
    @field_validator("tracker")
    @classmethod
    def _tracker_ok(cls, v: str):
        if v != "mediapipe-pose":
            raise ValueError("tracker must be 'mediapipe-pose'")
        return v

    @field_validator("t_ms")
    @classmethod
    def _nonneg_ts(cls, v: int):
        if v < 0:
            raise ValueError("t_ms must be >= 0")
        return v

    @field_validator("landmarks")
    @classmethod
    def _validate_lm_33(cls, v: List[Landmark]):
        if len(v) != 33:
            raise ValueError("landmarks must have 33 items")
        return v

    @field_validator("world_landmarks")
    @classmethod
    def _validate_wlm_33(cls, v: List[WorldLandmark]):
        if len(v) != 33:
            raise ValueError("world_landmarks must have 33 items")
        return v

# ---------- Health ----------
@app.get("/api/health")
async def health():
    return {"ok": True}

# ---------- WS ingest ----------
BATCH_SIZE = 30
BATCH_MS = 1000

# 5) improved thresholds
WARN_THRESHOLD = 0.70
RATE_THRESHOLD = 0.85
CRIT_THRESHOLD = 0.95  # advisory only

MAX_QUEUE = 300

# 9) file robustness: daily rotation into a dir; create if missing
DATA_DIR = "/data"
DATA_BASENAME = "pose_ingest"
def _data_path_for_today() -> str:
    day = datetime.now(timezone.utc).strftime("%Y%m%d")
    return os.path.join(DATA_DIR, f"{DATA_BASENAME}-{day}.jsonl")

logging.basicConfig(level=os.getenv("LOG_LEVEL", "DEBUG"))

@app.websocket("/api/pose/v1/ws")
async def ws_pose(websocket: WebSocket):
    await websocket.accept()

    queue: asyncio.Queue[PoseFrameV2] = asyncio.Queue(MAX_QUEUE)
    stop = asyncio.Event()
    last_rate_sent_level = False
    dropped_frames = 0  # 4) track counter

    async def writer():
        batch: list[PoseFrameV2] = []
        loop = asyncio.get_running_loop()  # 8)
        deadline = loop.time() + (BATCH_MS / 1000)
        os.makedirs(DATA_DIR, exist_ok=True)

        current_path = _data_path_for_today()
        try:
            f = open(current_path, "a", encoding="utf-8")
        except OSError as e:
            await _safe_send(websocket, {"error": "io_error", "detail": str(e)})
            stop.set()
            return

        try:
            while not stop.is_set():
                # 9) daily rotation
                new_path = _data_path_for_today()
                if new_path != current_path:
                    try:
                        f.flush()
                        f.close()
                        current_path = new_path
                        f = open(current_path, "a", encoding="utf-8")
                    except OSError as e:
                        await _safe_send(websocket, {"error": "io_error", "detail": str(e)})
                        stop.set()
                        break

                timeout = max(0, deadline - loop.time())
                try:
                    fr = await asyncio.wait_for(queue.get(), timeout=timeout)
                    batch.append(fr)
                    if len(batch) >= BATCH_SIZE:
                        deadline = loop.time() + (BATCH_MS / 1000)
                except asyncio.TimeoutError:
                    pass

                if batch and (len(batch) >= BATCH_SIZE or loop.time() >= deadline):
                    record = {
                        # 7) timezone-aware timestamps
                        "received_at": datetime.now(timezone.utc).isoformat(),
                        # keep original frame schemas; do not force to 'pose.v2'
                        "packet": {
                            "tracker": "mediapipe-pose",
                            "frames": [b.model_dump() for b in batch],  # 2)
                        },
                    }
                    try:
                        f.write(json.dumps(record, ensure_ascii=False) + "\n")
                        f.flush()
                    except OSError as e:
                        await _safe_send(websocket, {"error": "io_error", "detail": str(e)})
                        stop.set()
                        break
                    batch.clear()
                    deadline = loop.time() + (BATCH_MS / 1000)
        finally:
            # 3) flush on shutdown
            if batch:
                record = {
                    "received_at": datetime.now(timezone.utc).isoformat(),
                    "packet": {
                        "tracker": "mediapipe-pose",
                        "frames": [b.model_dump() for b in batch],
                    },
                }
                try:
                    f.write(json.dumps(record, ensure_ascii=False) + "\n")
                    f.flush()
                except OSError:
                    pass
            with contextlib.suppress(Exception):
                f.close()

    async def status():
        # periodic status to inform client of queue and drops (4)
        while not stop.is_set():
            await asyncio.sleep(10)
            q = queue.qsize()
            await _safe_send(websocket, {"type": "status", "queued": q, "dropped": dropped_frames})

    writer_task = asyncio.create_task(writer())
    status_task = asyncio.create_task(status())

    try:
        ping_task = asyncio.create_task(_ping(websocket))
        while True:
            msg = await websocket.receive_json()
            try:
                frame = PoseFrameV2(**msg)
            except ValidationError as e:
                await websocket.send_json({"error": "invalid_frame", "detail": str(e)})
                continue

            # 6) move per-frame log to DEBUG
            try:
                lm0 = frame.landmarks[0]
                logging.debug(
                    "[WS] schema=%s t=%d lm0=(%.3f,%.3f,%.3f)",
                    frame.schema, frame.t_ms, lm0.x, lm0.y, lm0.z
                )
            except Exception:
                pass

            try:
                queue.put_nowait(frame)
            except asyncio.QueueFull:
                dropped_frames += 1  # 4)
                # inform client immediately when dropping
                await _safe_send(websocket, {"warning": "dropped_frame", "queued": queue.qsize(), "dropped": dropped_frames})
                continue

            q = queue.qsize()

            # 5) warn earlier
            if q > int(MAX_QUEUE * WARN_THRESHOLD):
                await _safe_send(websocket, {"warning": "backpressure", "queued": q, "dropped": dropped_frames})

            # 5) rate hint near capacity
            if q > int(MAX_QUEUE * RATE_THRESHOLD) and not last_rate_sent_level:
                await _safe_send(websocket, {"type": "rate", "hz": 15, "queued": q, "dropped": dropped_frames})
                last_rate_sent_level = True
            elif q <= int(MAX_QUEUE * RATE_THRESHOLD) and last_rate_sent_level:
                last_rate_sent_level = False

            # 5) critical advisory (no hard throttle)
            if q > int(MAX_QUEUE * CRIT_THRESHOLD):
                await _safe_send(websocket, {"warning": "critical_backpressure", "queued": q, "dropped": dropped_frames})

    except WebSocketDisconnect:
        pass
    finally:
        stop.set()
        await asyncio.gather(writer_task, status_task, return_exceptions=True)
        with contextlib.suppress(asyncio.CancelledError):
            ping_task.cancel()

async def _ping(ws: WebSocket):
    while True:
        await asyncio.sleep(20)
        try:
            await ws.send_json({"type": "ping", "t": datetime.now(timezone.utc).isoformat()})  # 7)
        except RuntimeError:
            return

async def _safe_send(ws: WebSocket, payload: dict):
    with contextlib.suppress(RuntimeError):
        await ws.send_json(payload)
