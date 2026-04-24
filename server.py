#!/usr/bin/env python3
"""GDL90 flight sim — HTTP front-end + UDP back-end.

Run with: python3 scripts/gdl90-sim/server.py [--target 127.0.0.1:4000] [--port 8765]
Then open http://127.0.0.1:8765 in a browser.

The browser does physics + rendering; it POSTs ownship state to /pos a few
times per second, and the server encodes GDL90 frames and sends them via
UDP to the configured target (default: iOS Simulator on 127.0.0.1:4000).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gdl90


STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MIME = {
    ".html": "text/html; charset=utf-8",
    ".js":   "application/javascript; charset=utf-8",
    ".css":  "text/css; charset=utf-8",
    ".svg":  "image/svg+xml",
    ".json": "application/json",
    ".png":  "image/png",
    ".ico":  "image/x-icon",
}

# Updated at startup + on each /pos if we haven't yet. Also broadcast heartbeats
# from a background thread once UDP target is known.
_state = {
    "target": ("127.0.0.1", 4000),
    "last_pos_at": 0.0,
    "heartbeat_thread_started": False,
}


def _fetch_vfr_cycle() -> str:
    """Ask tachbase.com for the current VFR AIRAC cycle. Fall back to '2604'."""
    try:
        req = urllib.request.Request(
            "https://www.tachbase.com/cgi-bin/tile_manifest.py",
            headers={"User-Agent": "gdl90-sim"},
        )
        with urllib.request.urlopen(req, timeout=4) as r:
            data = json.loads(r.read().decode("utf-8"))
            cycle = data.get("cycles", {}).get("vfr", {}).get("cycle")
            if cycle:
                return str(cycle)
    except Exception as e:
        print(f"[sim] manifest fetch failed ({e}); using fallback cycle 2604", file=sys.stderr)
    return "2604"


_VFR_CYCLE = None  # lazy-filled


def _send_udp(pkt: bytes) -> None:
    host, port = _state["target"]
    try:
        _sock.sendto(pkt, (host, port))
    except OSError as e:
        print(f"[sim] UDP send failed to {host}:{port}: {e}", file=sys.stderr)


_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _heartbeat_loop() -> None:
    """Emit heartbeat at 1 Hz. Only runs while we have a recent /pos; otherwise
    the receiver would show "GPS valid" with a stale position, which is worse
    than silence."""
    while True:
        time.sleep(1.0)
        if time.time() - _state["last_pos_at"] < 3.0:
            _send_udp(gdl90.heartbeat(gps_valid=True))


class SimHandler(BaseHTTPRequestHandler):
    # Silence the default access log
    def log_message(self, fmt, *args):
        return

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: str):
        rel = path.lstrip("/")
        if rel.startswith("static/"):
            rel = rel[len("static/"):]
        full = os.path.normpath(os.path.join(STATIC_DIR, rel))
        if not full.startswith(STATIC_DIR) or not os.path.isfile(full):
            self.send_error(404)
            return
        ext = os.path.splitext(full)[1].lower()
        ctype = MIME.get(ext, "application/octet-stream")
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            return self._send_file("index.html")
        if path == "/config":
            global _VFR_CYCLE
            if _VFR_CYCLE is None:
                _VFR_CYCLE = _fetch_vfr_cycle()
            host, port = _state["target"]
            return self._send_json({
                "vfr_cycle": _VFR_CYCLE,
                "target": f"{host}:{port}",
            })
        return self._send_file(path)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b""
        try:
            data = json.loads(raw.decode("utf-8")) if raw else {}
        except json.JSONDecodeError:
            return self._send_json({"error": "invalid json"}, status=400)

        if path == "/pos":
            try:
                lat = float(data["lat"])
                lon = float(data["lon"])
                alt = float(data.get("alt", 0.0))
                hdg = float(data.get("hdg", 0.0))
                gs  = float(data.get("gs", 0.0))
                vs  = float(data.get("vs", 0.0))
            except (KeyError, TypeError, ValueError):
                return self._send_json({"error": "need lat/lon/alt/hdg/gs/vs"}, status=400)

            callsign = str(data.get("callsign", "SIM"))[:8]
            _state["last_pos_at"] = time.time()

            _send_udp(gdl90.ownship_report(lat, lon, alt, hdg, gs, vs, callsign=callsign))
            _send_udp(gdl90.ownship_geo_altitude(alt))

            if not _state["heartbeat_thread_started"]:
                _state["heartbeat_thread_started"] = True
                t = threading.Thread(target=_heartbeat_loop, daemon=True)
                t.start()

            return self._send_json({"ok": True})

        if path == "/target":
            try:
                host = str(data["host"])
                port = int(data["port"])
            except (KeyError, TypeError, ValueError):
                return self._send_json({"error": "need host/port"}, status=400)
            _state["target"] = (host, port)
            print(f"[sim] UDP target -> {host}:{port}")
            return self._send_json({"ok": True, "target": f"{host}:{port}"})

        return self.send_error(404)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="127.0.0.1:4000",
                        help="UDP destination host:port (iOS Simulator default)")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    args = parser.parse_args()

    host, _, port = args.target.partition(":")
    _state["target"] = (host or "127.0.0.1", int(port or "4000"))

    server = ThreadingHTTPServer((args.host, args.port), SimHandler)
    print(f"[sim] http://{args.host}:{args.port}")
    print(f"[sim] UDP target {_state['target'][0]}:{_state['target'][1]}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
