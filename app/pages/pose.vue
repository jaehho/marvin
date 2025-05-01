<template>
  <div class="simple_pose_container">
    <h2>Pose Landmark Detection</h2>
    <!-- 2x2 Grid Container -->
    <div class="grid_container">
      <!-- Top Left: Local Video with Overlay -->
      <div class="webcam">
        <h3>Webcam</h3>
        <div class="video_wrapper">
          <video ref="webcam" autoplay playsinline></video>
          <canvas ref="webcam_overlay_canvas" class="webcam_overlay_canvas"></canvas>
        </div>
      </div>
      <!-- Top Right: World Landmark 1 (XZ) -->
      <div class="world_landmark">
        <h3>World Landmark (XZ Plane)</h3>
        <canvas ref="world_canvas_xz" class="world_canvas"></canvas>
      </div>
      <!-- Bottom Left: World Landmark 2 (YZ) -->
      <div class="world_landmark">
        <h3>World Landmark (YZ Plane)</h3>
        <canvas ref="world_canvas_yz" class="world_canvas"></canvas>
      </div>
      <!-- Bottom Right: World Landmark 3 (XY) -->
      <div class="world_landmark">
        <h3>World Landmark (XY Plane)</h3>
        <canvas ref="world_canvas_xy" class="world_canvas"></canvas>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onBeforeUnmount } from "vue";
import {
  PoseLandmarker,
  FilesetResolver,
  DrawingUtils,
} from "@mediapipe/tasks-vision";

interface Landmark {
  x: number;
  y: number;
  z: number;
}

// Reactive State & Refs
const poseDetector = ref<any>(null);
const isDetectionLoopRunning = ref(false);
const animationFrameId = ref<number | null>(null); // For tracking animation frame

// Template element refs
const webcam = ref<HTMLVideoElement | null>(null);
const webcam_overlay_canvas = ref<HTMLCanvasElement | null>(null);
const world_canvas_xz = ref<HTMLCanvasElement | null>(null);
const world_canvas_yz = ref<HTMLCanvasElement | null>(null);
const world_canvas_xy = ref<HTMLCanvasElement | null>(null);

// Helper Functions
const clearCanvas = (canvas: HTMLCanvasElement | null) => {
  canvas?.getContext("2d")?.clearRect(0, 0, canvas.width, canvas.height);
};

const setCanvasSize = (canvas: HTMLCanvasElement, video: HTMLVideoElement) => {
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;
};

// Returns the projected coordinates for a given landmark on the specified plane.
const orthogonalProjection = (
  landmark: Landmark,
  plane: "xy" | "xz" | "yz",
  width: number,
  height: number
) => {
  const centerX = width / 2;
  const centerY = height / 2;

  let posX = 0,
    posY = 0;
  let shiftY = 0;

  switch (plane) {
    case "xy":
      posX = landmark.x;
      posY = landmark.y;
      shiftY = 150;
      break;
    case "xz":
      posX = landmark.x;
      posY = landmark.z;
      shiftY = 75;
      break;
    case "yz":
      posX = landmark.z;
      posY = landmark.y;
      shiftY = 150;
      break;
  }
  return { x: posX * width + centerX, y: posY * height + centerY + shiftY };
};

const keyLandmarkIndices = new Set([11, 12, 13, 14, 15, 16, 23, 24]);

// Draws world landmarks on a canvas using the specified coordinate plane.
function drawOrthogonalLandmarks(
  worldLandmarks: Landmark[][],
  canvas: HTMLCanvasElement,
  plane: "xy" | "xz" | "yz"
) {
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    console.error("Failed to get canvas context");
    return;
  }
  const { width, height } = canvas;
  if (webcam.value) {
    setCanvasSize(canvas, webcam.value);
  }
  ctx.lineJoin = "round";
  ctx.lineCap = "round";

  ctx.clearRect(0, 0, width, height);

  if (!worldLandmarks.length) return;

  worldLandmarks.forEach((landmarks) => {
    ctx.fillStyle = "red";
    keyLandmarkIndices.forEach((index) => {
      if (index < landmarks.length) {
        const { x, y } = orthogonalProjection(
          landmarks[index],
          plane,
          width,
          height
        );
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fill();
      }
    });

    ctx.strokeStyle = "blue";
    ctx.lineWidth = 3;
    PoseLandmarker.POSE_CONNECTIONS.forEach(({ start, end }) => {
      if (
        keyLandmarkIndices.has(start) &&
        keyLandmarkIndices.has(end) &&
        landmarks[start] &&
        landmarks[end]
      ) {
        const startPt = orthogonalProjection(
          landmarks[start],
          plane,
          width,
          height
        );
        const endPt = orthogonalProjection(
          landmarks[end],
          plane,
          width,
          height
        );
        ctx.beginPath();
        ctx.moveTo(startPt.x, startPt.y);
        ctx.lineTo(endPt.x, endPt.y);
        ctx.stroke();
      }
    });
  });
}

// Pose Detection Initialization
async function initializePoseDetection(
  runningMode: "IMAGE" | "VIDEO" = "VIDEO",
  numPoses: number = 1
) {
  const vision = await FilesetResolver.forVisionTasks(
    "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.0/wasm"
  );
  const poseLandmarker = await PoseLandmarker.createFromOptions(vision, {
    baseOptions: {
      modelAssetPath:
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task",
      delegate: "GPU",
    },
    runningMode,
    numPoses,
  });

  async function detectPoseOnVideo(
    video: HTMLVideoElement,
    canvas: HTMLCanvasElement,
    timestamp?: number
  ) {
    const ctx = canvas.getContext("2d")!;
    setCanvasSize(canvas, video);
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const ts = timestamp || performance.now();
    const result = await poseLandmarker.detectForVideo(video, ts);
    const drawingUtils = new DrawingUtils(ctx);

    result.landmarks.forEach((landmarks: any) => {
      drawingUtils.drawLandmarks(landmarks, {
        radius: (data: { from?: { z?: number } }) =>
          DrawingUtils.lerp(data.from?.z ?? 0, -0.15, 0.1, 5, 1),
      });
      drawingUtils.drawConnectors(landmarks, PoseLandmarker.POSE_CONNECTIONS);
    });
    return result;
  }

  return { poseLandmarker, detectPoseOnVideo };
}

// Pose Detection Loop
async function startPoseDetectionLoop() {
  if (isDetectionLoopRunning.value) return;
  isDetectionLoopRunning.value = true;
  if (
    !webcam.value ||
    !webcam_overlay_canvas.value ||
    !world_canvas_xz.value ||
    !world_canvas_yz.value ||
    !world_canvas_xy.value
  )
    return;

  const video = webcam.value;
  const overlay = webcam_overlay_canvas.value;
  const orthoCanvases = [
    world_canvas_xz.value,
    world_canvas_yz.value,
    world_canvas_xy.value,
  ];

  async function loop() {
    if (video.readyState >= video.HAVE_ENOUGH_DATA) {
      try {
        const result = await poseDetector.value.detectPoseOnVideo(
          video,
          overlay,
          performance.now()
        );

        // Draw landmarks on orthogonal canvases
        drawOrthogonalLandmarks(result.worldLandmarks, orthoCanvases[0], "xz");
        drawOrthogonalLandmarks(result.worldLandmarks, orthoCanvases[1], "yz");
        drawOrthogonalLandmarks(result.worldLandmarks, orthoCanvases[2], "xy");
      } catch (error) {
        console.error("Pose detection error:", error);
      }
    }
    animationFrameId.value = requestAnimationFrame(loop);
  }
  loop();
}

// Lifecycle Hook
onMounted(async () => {
  poseDetector.value = await initializePoseDetection("VIDEO", 1);
  initializeLocalStream();
});

async function initializeLocalStream() {
  try {
    const localMediaStream = await navigator.mediaDevices.getUserMedia({
      video: true,
    });
    if (webcam.value) {
      webcam.value.srcObject = localMediaStream;
    }
    startPoseDetectionLoop();
  } catch (error) {
    console.error("Media device error:", error);
  }
}

// Cleanup on page unload or refresh
onBeforeUnmount(() => {
  // Stop webcam stream
  const stream = webcam.value?.srcObject as MediaStream;
  stream?.getTracks().forEach((track) => track.stop());

  // Cancel animation frame loop
  if (animationFrameId.value !== null) {
    cancelAnimationFrame(animationFrameId.value);
  }

  // Clear canvases
  clearCanvas(webcam_overlay_canvas.value);
  clearCanvas(world_canvas_xz.value);
  clearCanvas(world_canvas_yz.value);
  clearCanvas(world_canvas_xy.value);

  // Dispose of poseLandmarker if method exists
  poseDetector.value?.poseLandmarker?.close?.();
});
</script>

<style scoped>
.simple_pose_container {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 20px;
}

.grid_container {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  grid-gap: 20px;
  justify-items: center;
  align-items: center;
}

.webcam,
.world_landmark {
  text-align: center;
}

.video_wrapper {
  position: relative;
  width: 400px;
  height: 300px;
}

video {
  width: 400px;
  height: 300px;
  background: black;
  border: 1px solid #ccc;
  transform: scaleX(-1);
}

.webcam_overlay_canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 400px;
  height: 300px;
  pointer-events: none;
  transform: scaleX(-1);
}

.world_canvas {
  border: 1px solid #aaa;
  width: 400px;
  height: 300px;
  transform: scaleX(-1);
}
</style>
