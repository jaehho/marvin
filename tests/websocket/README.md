# websockets test

```bash
pip install aiortc aiohttp
```

on server with port 8080 open

```bash
python signaling_server.py
```

on peer 1

```bash
python peer.py # answer y or n
```

on peer 2

```bash
python peer.py # answer opposite of peer 1
```