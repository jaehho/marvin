<script setup>
import { ref, onMounted, onBeforeUnmount, computed } from 'vue';

const SIGNAL_URL = 'ws://24.193.235.114:9000/ws';
const localId = crypto.randomUUID();
const hubId = 'hub';
let pc, dc, ws;
const input = ref('');
const messages = ref([]);
const status = ref('available');  // Track the status of the system (available or busy)
const control = ref(null);  // Track which client has control of the channel
const queue = ref([]);  // Track the ordered queue of clients

function log(msg) {
  console.log(`[DEBUG] [${localId}] ${msg}`);
}

function sendMessage() {
  if (control.value === localId && dc && dc.readyState === 'open' && input.value.trim()) {
    dc.send(input.value);
    messages.value.push(`[me] ${input.value}`);
    input.value = '';
  } else if (control.value !== localId) {
    messages.value.push('[System] You must have control to send messages.');
    log('Cannot send message: You do not have control');
  } else {
    messages.value.push('[System] The system is busy or you cannot send an empty message.');
    log('Message sending failed: System is busy or message is empty');
  }
}

function handleButtonClick() {
  if (control.value === null && !queue.value.includes(localId)) {
    // No control and not in the queue: Claim control
    ws.send(JSON.stringify({ type: 'claim_control', from: localId }));
    log('Attempting to claim control');
  } else if (control.value !== localId && !queue.value.includes(localId)) {
    // Someone has control and client is not in the queue: Join the Queue
    ws.send(JSON.stringify({ type: 'join_queue', from: localId }));
    log('Attempting to join the queue');
  } else if (control.value === localId) {
    // If the client has control: Give up control
    ws.send(JSON.stringify({ type: 'give_up_control', from: localId }));
    log('Giving up control');
  } else if (queue.value.includes(localId)) {
    // If the client is in the queue: Leave the queue
    ws.send(JSON.stringify({ type: 'leave_queue', from: localId }));
    log('Leaving the queue');
  }
}

const controlStatus = computed(() => {
  if (control.value === localId) {
    return 'You are in control';
  } else if (queue.value.includes(localId)) {
    return `You are in position ${queue.value.indexOf(localId) + 1} in the queue.`;
  } else if (control.value === null) {
    return 'Take control';
  } else {
    return 'Join queue';
  }
});

const dynamicButtonText = computed(() => {
  if (control.value === null && !queue.value.includes(localId)) {
    return 'Take Control'; // If neither control nor in queue, offer to take control
  } else if (control.value !== null && queue.value.includes(localId)) {
    return 'Leave Queue'; // If client is in queue, show "Leave Queue"
  } else if (control.value === localId) {
    return 'Give Up Control'; // If client has control, show "Give Up Control"
  } else {
    return 'Join Queue'; // Default if control is not held
  }
});

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
    } else if (data.type === 'status') {
      status.value = data.status;  // Update the client status based on the server's response
      control.value = data.control;  // Update who currently has control
      queue.value = data.queue;  // Update the queue with the ordered list
      log(`System status: ${status.value}, Control: ${control.value}, Queue: ${queue.value}`);
    } else if (data.type === 'queue_update') {
      // Process the updated queue and log the position correctly
      queue.value = data.queue.map(client => client.client_id);  // Update queue with only client_ids
      data.queue.forEach(client => {
        if (client.client_id === localId) {
          log(`Queue updated: You are in position ${client.queue_position}`);
        }
      });
    } else if (data.type === 'busy') {
      messages.value.push(data.message);  // Show the busy message to the user
    } else if (data.type === 'error') {
      messages.value.push(data.message);  // Show error messages to the user
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
  if (queue.value.includes(localId)) {
    handleButtonClick();  // Automatically take the appropriate action when leaving the page
  }
});
</script>

<template>
  <div>
    <h2>Spoke {{ localId }}</h2>
    <div v-for="(msg, idx) in messages" :key="idx">{{ msg }}</div>
    <input v-model="input" @keyup.enter="sendMessage" placeholder="Type a message..." />
    <button @click="sendMessage" :disabled="control !== localId">Send</button>

    <div>
      <button @click="handleButtonClick" :disabled="!['Take Control', 'Join Queue', 'Give Up Control', 'Leave Queue'].includes(dynamicButtonText)">
        {{ dynamicButtonText }}
      </button>
    </div>

    <div>
      <p>{{ controlStatus }}</p>
    </div>
  </div>
</template>
