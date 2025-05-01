<template>
  <div>
    <h2>Spoke {{ localId }}</h2>

    <div v-for="(msg, idx) in messages" :key="idx">{{ msg }}</div>

    <input
      v-model="input"
      @keyup.enter="sendMessage"
      placeholder="Type a message..."
    />
    <button @click="sendMessage" :disabled="clientMode !== 'in_control'">
      Send
    </button>

    <div style="margin-top: 1em">
      <button @click="handleButtonClick">
        {{ dynamicButtonText }}
      </button>
    </div>

    <div style="margin-top: 0.5em">
      <p>{{ controlStatus }}</p>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue';

const SIGNAL_URL = 'ws://24.193.235.114:9000/ws';
const localId = crypto.randomUUID();
const hubId = 'hub';

let pc, dc, ws;

const input = ref('');
const messages = ref([]);
const status = ref('available');
const control = ref(null);
const queue = ref([]);

function log(msg) {
  console.log(`[DEBUG] [${localId}] ${msg}`);
}

const clientMode = computed(() => {
  if (control.value === localId) return 'in_control';
  if (queue.value.includes(localId)) return 'in_queue';
  if (control.value === null) return 'idle';
  return 'others_control';
});

const modeConfig = {
  idle: {
    buttonText: 'Take Control',
    statusText: 'You can take control',
    actionType: 'claim_control'
  },
  others_control: {
    buttonText: 'Join Queue',
    statusText: 'You can join queue',
    actionType: 'join_queue'
  },
  in_control: {
    buttonText: 'Give Up Control',
    statusText: 'You are in control',
    actionType: 'give_up_control'
  },
  in_queue: {
    buttonText: 'Leave Queue',
    statusText: () => `You are in position ${queue.value.indexOf(localId) + 1} in the queue.`,
    actionType: 'leave_queue'
  }
};

const dynamicButtonText = computed(() => modeConfig[clientMode.value].buttonText);
const controlStatus = computed(() => {
  const txt = modeConfig[clientMode.value].statusText;
  return typeof txt === 'function' ? txt() : txt;
});

function handleButtonClick() {
  const { actionType } = modeConfig[clientMode.value];
  ws.send(JSON.stringify({ type: actionType, from: localId }));
  log(`Sent ${actionType}`);
}

function sendMessage() {
  if (clientMode.value !== 'in_control') {
    messages.value.push('[System] You must have control to send messages.');
    log('Cannot send: not in control');
    return;
  }
  if (!input.value.trim()) {
    messages.value.push('[System] Cannot send an empty message.');
    log('Cannot send: empty');
    return;
  }
  dc.send(input.value);
  messages.value.push(`[me] ${input.value}`);
  input.value = '';
}

function handleUnload() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    if (control.value === localId) {
      ws.send(JSON.stringify({ type: 'give_up_control', from: localId }));
    } else if (queue.value.includes(localId)) {
      ws.send(JSON.stringify({ type: 'leave_queue', from: localId }));
    }
  }
}

onMounted(async () => {
  window.addEventListener('beforeunload', handleUnload);

  pc = new RTCPeerConnection({
    iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
  });

  ws = new WebSocket(SIGNAL_URL);
  ws.onopen = async () => {
    log('Connected to signaling server');
    ws.send(JSON.stringify({ type: 'register', id: localId }));

    dc = pc.createDataChannel('chat');
    dc.onopen = () => log('DataChannel opened');
    dc.onmessage = e => {
      messages.value.push(e.data);
      log(`Received: ${e.data}`);
    };

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    ws.send(JSON.stringify({
      type: 'offer',
      sdp: offer.sdp,
      from: localId,
      to: hubId
    }));
    log('Sent offer to hub');
  };

  ws.onmessage = async (event) => {
    const data = JSON.parse(event.data);
    switch (data.type) {
      case 'answer':
        await pc.setRemoteDescription(data);
        log('Received answer');
        break;

      case 'candidate':
        await pc.addIceCandidate(data.candidate);
        log('Added ICE candidate');
        break;

      case 'status':
        status.value = data.status;
        control.value = data.control;
        queue.value = data.queue;
        log(`Status update: control=${control.value}, queue=[${queue.value}]`);
        break;

      case 'queue_update':
        queue.value = data.queue.map(c => c.client_id);
        data.queue.forEach(c => {
          if (c.client_id === localId) {
            log(`Queue position: ${c.queue_position}`);
          }
        });
        break;

      case 'busy':
      case 'error':
        messages.value.push(`[System] ${data.message}`);
        log(`${data.type.toUpperCase()}: ${data.message}`);
        break;
    }
  };

  pc.onicecandidate = ({ candidate }) => {
    if (candidate && ws.readyState === 1) {
      ws.send(JSON.stringify({
        type: 'candidate',
        candidate,
        from: localId,
        to: hubId
      }));
      log('Sent ICE candidate');
    }
  };

  pc.onconnectionstatechange = () => {
    log(`Connection state: ${pc.connectionState}`);
  };
});

onBeforeUnmount(() => {
  window.removeEventListener('beforeunload', handleUnload);
  ws?.close();
  pc?.close();
});
</script>
