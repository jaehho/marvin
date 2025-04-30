<script setup>
import { ref, onMounted, onBeforeUnmount } from 'vue';

const SIGNAL_URL = 'ws://24.193.235.114:9000/ws';
const localId = crypto.randomUUID();
const hubId = 'hub';
let pc, dc, ws;
const input = ref('');
const messages = ref([]);

function log(msg) {
  console.log(`[${localId}] ${msg}`);
}

function sendMessage() {
  if (dc && dc.readyState === 'open' && input.value.trim()) {
    dc.send(input.value);
    messages.value.push(`[me] ${input.value}`);
    input.value = '';
  }
}

onMounted(async () => {
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
    if (data.type === 'answer') {
      await pc.setRemoteDescription(data);
      log('Received answer');
    } else if (data.type === 'candidate') {
      await pc.addIceCandidate(data.candidate);
      log('Added ICE candidate');
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
  ws?.close();
  pc?.close();
});
</script>

<template>
  <div>
    <h2>Spoke {{ localId }}</h2>
    <div v-for="(msg, idx) in messages" :key="idx">{{ msg }}</div>
    <input v-model="input" @keyup.enter="sendMessage" placeholder="Type a message..." />
    <button @click="sendMessage">Send</button>
  </div>
</template>
