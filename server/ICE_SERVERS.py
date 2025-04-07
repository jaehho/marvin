# ice_config.py
from aiortc import RTCIceServer

ICE_SERVERS = [
    RTCIceServer(
        urls="turn:your.turn.server:3478",
        username="your-username",
        credential="your-password"
    )
]
