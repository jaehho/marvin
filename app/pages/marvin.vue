<template>
    <div>
      <h2>Spoke {{ localId }} – Pose Client</h2>
  
      <!-- control / queue UI -->
      <div>
        <button @click="handleButtonClick">
          {{ dynamicButtonText }}
        </button>
        <span style="margin-left: 1em">{{ controlStatus }}</span>
      </div>
  
      <!-- pose detection UI -->
      <div class="simple_pose_container">
        <h3>Webcam + Pose Overlay</h3>
        <div class="video_wrapper">
          <video ref="webcam" autoplay playsinline></video>
          <canvas ref="webcam_overlay_canvas" class="webcam_overlay_canvas"></canvas>
        </div>
  
        <div class="world_landmarks">
          <h4>World XZ</h4>
          <canvas ref="world_canvas_xz" class="world_canvas"></canvas>
        </div>
        <div class="world_landmarks">
          <h4>World YZ</h4>
          <canvas ref="world_canvas_yz" class="world_canvas"></canvas>
        </div>
        <div class="world_landmarks">
          <h4>World XY</h4>
          <canvas ref="world_canvas_xy" class="world_canvas"></canvas>
        </div>
      </div>
    </div>
  </template>
  
  <script setup lang="ts">
  import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
  import { PoseLandmarker, FilesetResolver, DrawingUtils } from '@mediapipe/tasks-vision'
  
  /** —————————————————————————
   *  Signaling / WebRTC setup
   *  ————————————————————————— */
  const SIGNAL_URL = 'ws://24.193.235.114:9000/ws'
  const localId = crypto.randomUUID()
  const hubId = 'hub'
  
  let pc: RTCPeerConnection
  let dc: RTCDataChannel
  let ws: WebSocket
  
  const control = ref<string|null>(null)
  const queue = ref<string[]>([])
  
  function log(msg: string) {
    console.log(`[DEBUG][${localId}] ${msg}`)
  }
  
  const clientMode = computed(() => {
    if (control.value === localId) return 'in_control'
    if (queue.value.includes(localId)) return 'in_queue'
    if (control.value === null) return 'idle'
    return 'others_control'
  })
  
  const modeConfig = {
    idle:  { buttonText: 'Take Control',    statusText: 'You can take control',  actionType: 'claim_control' },
    others_control: { buttonText: 'Join Queue', statusText: 'You can join queue',    actionType: 'join_queue' },
    in_control:     { buttonText: 'Give Up Control', statusText: 'You are in control', actionType: 'give_up_control' },
    in_queue:       { buttonText: 'Leave Queue', statusText: () => `You are #${queue.value.indexOf(localId)+1} in queue`, actionType: 'leave_queue' }
  }
  
  const dynamicButtonText = computed(() => modeConfig[clientMode.value].buttonText)
  const controlStatus     = computed(() => {
    const t = modeConfig[clientMode.value].statusText
    return typeof t === 'function' ? t() : t
  })
  
  function handleButtonClick() {
    const { actionType } = modeConfig[clientMode.value]
    ws.send(JSON.stringify({ type: actionType, from: localId }))
    log(`Sent ${actionType}`)
  }
  
  function handleUnload() {
    if (ws?.readyState === WebSocket.OPEN) {
      const { actionType } = modeConfig[clientMode.value]
      if (actionType === 'give_up_control' || actionType === 'leave_queue') {
        ws.send(JSON.stringify({ type: actionType, from: localId }))
      }
    }
  }
  
  onMounted(async () => {
    window.addEventListener('beforeunload', handleUnload)
  
    // 1. create RTCPeerConnection + DataChannel
    pc = new RTCPeerConnection({ iceServers: [{ urls: 'stun:stun.l.google.com:19302' }] })
  
    // 2. connect to signaling server
    ws = new WebSocket(SIGNAL_URL)
    ws.onopen = async () => {
      log('WS open – registering')
      ws.send(JSON.stringify({ type: 'register', id: localId }))
  
      // create our outbound data channel
      dc = pc.createDataChannel('pose')
      dc.onopen    = () => log('DataChannel opened')
      dc.onmessage = e => log(`→ hub replied: ${e.data}`)
  
      // offer/answer handshake
      const offer = await pc.createOffer()
      await pc.setLocalDescription(offer)
      ws.send(JSON.stringify({
        type: 'offer', sdp: offer.sdp, from: localId, to: hubId
      }))
      log('Sent offer')
    }
  
    ws.onmessage = async evt => {
      const msg = JSON.parse(evt.data)
      switch (msg.type) {
        case 'answer':
          await pc.setRemoteDescription(msg)
          log('Got answer')
          break
        case 'candidate':
          await pc.addIceCandidate(msg.candidate)
          log('Added ICE candidate')
          break
        case 'status':
          control.value = msg.control
          queue.value   = msg.queue
          break
        case 'queue_update':
          queue.value = msg.queue.map((c: any) => c.client_id)
          break
        case 'error':
        case 'busy':
          console.warn(msg.message)
          break
      }
    }
  
    pc.onicecandidate = e => {
      if (e.candidate && ws.readyState === 1) {
        ws.send(JSON.stringify({
          type: 'candidate',
          candidate: e.candidate,
          from: localId, to: hubId
        }))
      }
    }
  
    pc.onconnectionstatechange = () => {
      log(`PeerConnection state: ${pc.connectionState}`)
    }
  
    /** —————————————————————————
     *  Pose detection setup
     *  ————————————————————————— */
    const webcam = ref<HTMLVideoElement|null>(null)
    const overlay = ref<HTMLCanvasElement|null>(null)
    const worldXZ = ref<HTMLCanvasElement|null>(null)
    const worldYZ = ref<HTMLCanvasElement|null>(null)
    const worldXY = ref<HTMLCanvasElement|null>(null)
  
    // key landmarks we’ll send
    const keyIndices = new Set([11,12,13,14,15,16,23,24])
  
    // init MediaPipe
    const poseDetector = ref<any>(null)
    async function initPose() {
      const vision = await FilesetResolver.forVisionTasks(
        'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.0/wasm'
      )
      poseDetector.value = await PoseLandmarker.createFromOptions(vision, {
        baseOptions: {
          modelAssetPath:
            'https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task',
          delegate: 'GPU'
        },
        runningMode: 'VIDEO',
        numPoses: 1
      })
    }
  
    function setCanvasSize(c: HTMLCanvasElement, v: HTMLVideoElement) {
      c.width  = v.videoWidth
      c.height = v.videoHeight
    }
  
    function drawWorld(worldLandmarks: any[], canvas: HTMLCanvasElement, plane: 'xz'|'yz'|'xy') {
      const ctx = canvas.getContext('2d')!
      const w = canvas.width, h = canvas.height
      ctx.clearRect(0,0,w,h)
      worldLandmarks.forEach(lms => {
        // draw key points
        keyIndices.forEach(i => {
          if (!lms[i]) return
          let x=0,y=0
          if (plane==='xz') { x = (lms[i].x * w) + w/2; y = (lms[i].z * h) + h/2 }
          if (plane==='yz') { x = (lms[i].z * w) + w/2; y = (lms[i].y * h) + h/2 }
          if (plane==='xy') { x = (lms[i].x * w) + w/2; y = (lms[i].y * h) + h/2 }
          ctx.fillStyle = 'red'
          ctx.beginPath()
          ctx.arc(x,y,5,0,2*Math.PI)
          ctx.fill()
        })
      })
    }
  
    async function startDetection() {
      const video = webcam.value!
      const camCanvas = overlay.value!
      const ortho = [worldXZ.value!, worldYZ.value!, worldXY.value!]
  
      async function frameLoop(now: number) {
        if (video.readyState >= video.HAVE_ENOUGH_DATA && poseDetector.value) {
          // draw overlay
          setCanvasSize(camCanvas, video)
          const result = await poseDetector.value.detectForVideo(video, now)
          const dutils = new DrawingUtils(camCanvas.getContext('2d')!)
          result.landmarks.forEach((lm:any) => {
            dutils.drawConnectors(lm, PoseLandmarker.POSE_CONNECTIONS)
            dutils.drawLandmarks(lm)
          })
  
          // world views
          ortho.forEach((cnv,i) => {
            setCanvasSize(cnv, video)
          })
          drawWorld(result.worldLandmarks, ortho[0], 'xz')
          drawWorld(result.worldLandmarks, ortho[1], 'yz')
          drawWorld(result.worldLandmarks, ortho[2], 'xy')
  
          // if we have control & data-channel open → send key landmarks
          if (clientMode.value === 'in_control' && dc.readyState === 'open') {
            const wlm = result.worldLandmarks[0]||[]
            const payload: Record<number, {x:number,y:number,z:number}> = {}
            wlm.forEach((pt:any,i:number) => {
              if (keyIndices.has(i)) {
                payload[i] = { x:pt.x, y:pt.y, z:pt.z }
              }
            })
            dc.send(JSON.stringify({ type:'landmarks', from: localId, data: payload }))
            log(`Sent ${Object.keys(payload).length} key landmarks`)
          }
        }
        requestAnimationFrame(frameLoop)
      }
      frameLoop(performance.now())
    }
  
    // startup both in parallel
    await initPose()
    // start video
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true })
      webcam.value!.srcObject = stream
      webcam.value!.play()
      startDetection()
    } catch(e) {
      console.error(e)
    }
  })
  
  onBeforeUnmount(() => {
    window.removeEventListener('beforeunload', handleUnload)
    ws?.close()
    pc?.close()
  })
  </script>
  
  <style scoped>
  .simple_pose_container {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 1em;
    margin-top: 1em;
  }
  .video_wrapper {
    position: relative;
    width: 320px; height: 240px;
  }
  video, .webcam_overlay_canvas,
  .world_canvas {
    width: 320px; height: 240px;
    transform: scaleX(-1);
    border: 1px solid #ccc;
  }
  .webcam_overlay_canvas {
    position: absolute; top: 0; left: 0;
    pointer-events: none;
  }
  .world_landmarks { text-align: center; }
  </style>
  