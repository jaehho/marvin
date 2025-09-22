# app.py
from fastapi import FastAPI, Header, HTTPException
from pydantic import BaseModel, Field, validator
from typing import List, Optional, Literal
from datetime import datetime
import json, os

app = FastAPI(title="Pose Ingest API", version="1.0")

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

class PoseFrame(BaseModel):
    t_ms: int = Field(..., description="Monotonic or wall-clock timestamp in ms")
    landmarks: LandmarkList
    world_landmarks: Optional[WorldLandmarkList] = None

    @validator("t_ms")
    def nonnegative(cls, v):  # noqa: N805
        if v < 0:
            raise ValueError("t_ms must be >= 0")
        return v

class PosePacket(BaseModel):
    tracker: Optional[Literal["mediapipe-pose"]] = "mediapipe-pose"
    frames: List[PoseFrame]

# ---- Health ----
@app.get("/api/health")
def health():
    return {"ok": True}

# ---- Ingest ----
DATA_PATH = "/data/pose_ingest.jsonl"
os.makedirs(os.path.dirname(DATA_PATH), exist_ok=True)

@app.post("/api/pose/v1/ingest")
def ingest(packet: PosePacket, x_api_key: Optional[str] = Header(None)):
    # Optional simple auth: set API_KEY env to enforce
    expected = os.getenv("API_KEY")
    if expected and x_api_key != expected:
        raise HTTPException(status_code=401, detail="invalid api key")

    record = {
        "received_at": datetime.utcnow().isoformat() + "Z",
        "packet": packet.dict(),
    }
    with open(DATA_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")

    frames = len(packet.frames)
    points = len(packet.frames[0].landmarks) if frames else 0
    return {"status": "stored", "frames": frames, "landmarks_per_frame": points}

# ---- Minimal compute sample: average visibility per packet ----
@app.post("/api/pose/v1/avg_visibility")
def avg_visibility(packet: PosePacket):
    vis_vals = []
    for fr in packet.frames:
        for lm in fr.landmarks:
            if lm.visibility is not None:
                vis_vals.append(lm.visibility)
    avg = sum(vis_vals) / len(vis_vals) if vis_vals else None
    return {"avg_visibility": avg, "count": len(vis_vals)}
