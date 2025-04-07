# http test

```bash
pip install aiortc aiohttp
```

on server with port 8080 open

```bash
python signaling_server.py
```

on peer 1

```bash
python peer_offer.py # or python peer_answer.py
```

on peer 2

```bash
python peer_answer.py # or python peer_offer.py
```

order does not matter but seems to only work once and then you need to restart signaling server