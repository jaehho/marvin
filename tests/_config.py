from aiortc import RTCIceServer

TURN = RTCIceServer(
    urls="turn:24.193.235.114:3478",
    username="jaeho",
    credential="2121"
)
