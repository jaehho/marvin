# ice_config.py
from aiortc import RTCIceServer

ICE_SERVERS = [
    RTCIceServer(
        urls="turn:24.193.235.114:3478",
        username="jaeho",
        credential="2121"
    )
]
