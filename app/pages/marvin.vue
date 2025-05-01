<template>
  <div class="container">
    <!-- Control / Chat Panel -->
    <section class="chat-panel">
      <h2>Spoke {{ localId }}</h2>
      <div class="messages">
        <div v-for="(msg, idx) in messages" :key="idx" class="message">
          {{ msg }}
        </div>
      </div>
      <div class="controls">
        <button @click="handleButtonClick">{{ dynamicButtonText }}</button>
        <p>{{ controlStatus }}</p>
      </div>
    </section>

    <!-- Pose Detection Panel -->
    <section class="pose-panel">
      <h2>Pose Landmark Detection</h2>
      <div class="grid-container">
        <div class="webcam">
          <h3>Webcam</h3>
          <div class="video-wrapper">
            <video ref="webcam" autoplay playsinline></video>
            <canvas ref="webcamOverlay" class="overlay"></canvas>
          </div>
        </div>
        <div class="world">
          <h3>World XZ</h3>
          <canvas ref="canvasXZ" class="world-canvas"></canvas>
        </div>
        <div class="world">
          <h3>World YZ</h3>
          <canvas ref="canvasYZ" class="world-canvas"></canvas>
        </div>
        <div class="world">
          <h3>World XY</h3>
          <canvas ref="canvasXY" class="world-canvas"></canvas>
        </div>
      </div>
    </section>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onBeforeUnmount } from "vue";
import {
  PoseLandmarker,
  FilesetResolver,
  DrawingUtils,
} from "@mediapipe/tasks-vision";

// --- Signaling / WebRTC Chat Setup ---
const SIGNAL_URL = "ws://24.193.235.114:9000/ws";
const hubId = "hub";
const localId = crypto.randomUUID();
let pc: RTCPeerConnection;
let dc: RTCDataChannel;
let ws: WebSocket;

const messages = ref<string[]>([]);
const status = ref<"available" | "busy">("available");
const control = ref<string | null>(null);
const queue = ref<string[]>([]);

function log(msg: string) {
  console.log(`[DEBUG][${localId}] ${msg}`);
}

const clientMode = computed(() => {
  if (control.value === localId) return "in_control";
  if (queue.value.includes(localId)) return "in_queue";
  if (!control.value) return "idle";
  return "others_control";
});

const modeConfig = {
  idle: {
    buttonText: "Take Control",
    statusText: "You can take control",
    actionType: "claim_control",
  },
  others_control: {
    buttonText: "Join Queue",
    statusText: "You can join queue",
    actionType: "join_queue",
  },
  in_control: {
    buttonText: "Give Up Control",
    statusText: "You are in control",
    actionType: "give_up_control",
  },
  in_queue: {
    buttonText: "Leave Queue",
    statusText: () => `You are #${queue.value.indexOf(localId) + 1} in queue`,
    actionType: "leave_queue",
  },
};

const dynamicButtonText = computed(
  () => modeConfig[clientMode.value].buttonText
);
const controlStatus = computed(() => {
  const t = modeConfig[clientMode.value].statusText;
  return typeof t === "function" ? t() : t;
});

function handleButtonClick() {
  ws.send(
    JSON.stringify({
      type: modeConfig[clientMode.value].actionType,
      from: localId,
    })
  );
  log(`Sent ${modeConfig[clientMode.value].actionType}`);
}

function handleUnload() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    const mode = clientMode.value;
    if (mode === "in_control" || mode === "in_queue") {
      ws.send(
        JSON.stringify({ type: modeConfig[mode].actionType, from: localId })
      );
    }
  }
}

onMounted(async () => {
  window.addEventListener("beforeunload", handleUnload);

  // WebRTC PeerConnection
  pc = new RTCPeerConnection({
    iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
  });

  // Signaling WebSocket
  ws = new WebSocket(SIGNAL_URL);
  ws.onopen = async () => {
    log("Connected to signaling server");
    ws.send(JSON.stringify({ type: "register", id: localId }));
    // Create data channel
    dc = pc.createDataChannel("chat");
    dc.onopen = () => log("Data channel open");
    dc.onmessage = (e) => {
      messages.value.push(`[peer] ${e.data}`);
      log(`Received message: ${e.data}`);
    };
    // Offer/Answer handshake
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    ws.send(
      JSON.stringify({
        type: "offer",
        sdp: offer.sdp,
        from: localId,
        to: hubId,
      })
    );
    log("Sent offer");
  };

  ws.onmessage = async ({ data }) => {
    const msg = JSON.parse(data);
    switch (msg.type) {
      case "answer":
        await pc.setRemoteDescription(msg);
        log("Received answer");
        break;
      case "candidate":
        await pc.addIceCandidate(msg.candidate);
        log("Added ICE candidate");
        break;
      case "status":
        status.value = msg.status;
        control.value = msg.control;
        queue.value = msg.queue;
        log(`Status update: control=${control.value} queue=[${queue.value}]`);
        break;
      case "queue_update":
        queue.value = msg.queue.map((c: any) => c.client_id);
        break;
      case "error":
      case "busy":
        messages.value.push(`[System] ${msg.message}`);
        break;
    }
  };

  pc.onicecandidate = ({ candidate }) => {
    if (candidate && ws.readyState === 1) {
      ws.send(
        JSON.stringify({
          type: "candidate",
          candidate,
          from: localId,
          to: hubId,
        })
      );
      log("Sent ICE candidate");
    }
  };
});

onBeforeUnmount(() => {
  window.removeEventListener("beforeunload", handleUnload);
  ws?.close();
  pc?.close();
});

// --- Pose Detection Setup ---
interface Landmark {
  x: number;
  y: number;
  z: number;
}
const keyLandmarkIndices = new Set([11, 12, 13, 14, 15, 16, 23, 24]);

// Refs for templates
const webcam = ref<HTMLVideoElement | null>(null);
const webcamOverlay = ref<HTMLCanvasElement | null>(null);
const canvasXZ = ref<HTMLCanvasElement | null>(null);
const canvasYZ = ref<HTMLCanvasElement | null>(null);
const canvasXY = ref<HTMLCanvasElement | null>(null);

let poseLandmarker: PoseLandmarker, detectForVideo: any;
let animationId: number | null = null;

async function initPose() {
  const vision = await FilesetResolver.forVisionTasks(
    "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.0/wasm"
  );
  poseLandmarker = await PoseLandmarker.createFromOptions(vision, {
    baseOptions: {
      modelAssetPath:
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task",
      delegate: "GPU",
    },
    runningMode: "VIDEO",
    numPoses: 1,
  });
  detectForVideo = poseLandmarker.detectForVideo.bind(poseLandmarker);
}

function resizeCanvas(canvas: HTMLCanvasElement, video: HTMLVideoElement) {
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;
}

function orthographic(
  landmark: Landmark,
  plane: "xy" | "xz" | "yz",
  w: number,
  h: number
) {
  const cx = w / 2,
    cy = h / 2;
  let px = 0,
    py = 0,
    shiftY = 0;
  if (plane === "xy") {
    px = landmark.x;
    py = landmark.y;
    shiftY = 150;
  }
  if (plane === "xz") {
    px = landmark.x;
    py = landmark.z;
    shiftY = 75;
  }
  if (plane === "yz") {
    px = landmark.z;
    py = landmark.y;
    shiftY = 150;
  }
  return { x: px * w + cx, y: py * h + cy + shiftY };
}

function drawWorld(
  worldLandmarks: Landmark[][],
  canvas: HTMLCanvasElement,
  plane: "xy" | "xz" | "yz"
) {
  const ctx = canvas.getContext("2d")!;
  const v = webcam.value!;
  resizeCanvas(canvas, v);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  worldLandmarks.forEach((landmarks) => {
    // points
    ctx.fillStyle = "red";
    keyLandmarkIndices.forEach((i) => {
      if (landmarks[i]) {
        const p = orthographic(
          landmarks[i],
          plane,
          canvas.width,
          canvas.height
        );
        ctx.beginPath();
        ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
        ctx.fill();
      }
    });
    // connections
    ctx.strokeStyle = "blue";
    ctx.lineWidth = 3;
    PoseLandmarker.POSE_CONNECTIONS.forEach(({ start, end }) => {
      if (
        keyLandmarkIndices.has(start) &&
        keyLandmarkIndices.has(end) &&
        landmarks[start] &&
        landmarks[end]
      ) {
        const p1 = orthographic(
          landmarks[start],
          plane,
          canvas.width,
          canvas.height
        );
        const p2 = orthographic(
          landmarks[end],
          plane,
          canvas.width,
          canvas.height
        );
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
    });
  });
}

async function startLoop() {
  if (!webcam.value) return;
  const video = webcam.value;
  const overlay = webcamOverlay.value!;
  const orthoCanvases = [canvasXZ.value!, canvasYZ.value!, canvasXY.value!];

  async function loop() {
    if (video.readyState >= video.HAVE_ENOUGH_DATA) {
      const res = await detectForVideo(video, performance.now());
      // draw overlay
      {
        const ctx = overlay.getContext("2d")!;
        resizeCanvas(overlay, video);
        ctx.clearRect(0, 0, overlay.width, overlay.height);
        const du = new DrawingUtils(ctx);
        res.landmarks.forEach((l: any) => {
          du.drawLandmarks(l, {
            radius: (d) => DrawingUtils.lerp(d.from?.z || 0, -0.15, 0.1, 5, 1),
          });
          du.drawConnectors(l, PoseLandmarker.POSE_CONNECTIONS);
        });
      }
      // draw orthographic
      drawWorld(res.worldLandmarks, orthoCanvases[0], "xz");
      drawWorld(res.worldLandmarks, orthoCanvases[1], "yz");
      drawWorld(res.worldLandmarks, orthoCanvases[2], "xy");
      // send key landmarks if in control
      if (clientMode.value === "in_control" && dc.readyState === "open") {
        const keyData = res.worldLandmarks.map((landmarks) =>
          [...keyLandmarkIndices].map((i) => ({
            index: i,
            x: landmarks[i]?.x ?? null,
            y: landmarks[i]?.y ?? null,
            z: landmarks[i]?.z ?? null,
          }))
        );
        dc.send(
          JSON.stringify({ type: "landmarks", from: localId, data: keyData })
        );
        messages.value.push(`[me] sent ${keyData[0].length} key landmarks`);
      }
    }
    animationId = requestAnimationFrame(loop);
  }
  loop();
}

async function initCamera() {
  const stream = await navigator.mediaDevices.getUserMedia({ video: true });
  if (webcam.value) webcam.value.srcObject = stream;
  await startLoop();
}

onMounted(async () => {
  await initPose();
  await initCamera();
});

onBeforeUnmount(() => {
  animationId && cancelAnimationFrame(animationId);
  const tracks = (webcam.value?.srcObject as MediaStream)?.getTracks() || [];
  tracks.forEach((t) => t.stop());
  poseLandmarker?.close?.();
});
</script>

<style scoped>
.container {
  display: flex;
  flex-direction: row;
  gap: 2rem;
  padding: 1rem;
}
.chat-panel {
  flex: 1;
  border-right: 1px solid #ccc;
  padding-right: 1rem;
}
.messages {
  max-height: 60vh;
  overflow-y: auto;
  margin-bottom: 1rem;
}
.message {
  padding: 0.25rem 0;
}
.controls button {
  margin-bottom: 0.5rem;
}
.pose-panel {
  flex: 2;
}
.grid-container {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}
.video-wrapper {
  position: relative;
  width: 400px;
  height: 300px;
}
video,
.overlay,
.world-canvas {
  width: 400px;
  height: 300px;
  transform: scaleX(-1);
}

.overlay {
  position: absolute;
  top: 0;
  left: 0;
  pointer-events: none;
}
.world-canvas {
  border: 1px solid #aaa;
}
</style>
