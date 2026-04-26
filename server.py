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
import math
import os
import socket
import subprocess
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
    "target": ("255.255.255.255", 4000),
    "last_pos_at": 0.0,
    "heartbeat_thread_started": False,
    # Previous /pos sample used to derive bank angle from turn rate.
    "prev_hdg": None,
    "prev_t": 0.0,
}


def _detect_subnet_broadcast() -> str:
    """Return the broadcast address of the Mac's primary outbound interface.

    Mirrors what a Stratus does on its own AP: subnet broadcast on UDP 4000.
    Sending to the Wi-Fi subnet's broadcast address routes via the normal
    routing table, so it survives VPNs / Ethernet / virtual interfaces being
    up at the same time.

    Reads the actual broadcast address from `ifconfig` rather than guessing
    /24 — corporate Wi-Fi commonly uses /23 or /22, and a wrong-mask
    broadcast goes nowhere useful. Falls back to limited broadcast
    (255.255.255.255) if anything in the chain fails.
    """
    # Step 1: ask the routing table which IP we'd source a packet from. No
    # bytes go on the wire — connect on a UDP socket just resolves the
    # source IP via the routing rule for 8.8.8.8.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 1))
        local_ip = s.getsockname()[0]
    except OSError:
        return "255.255.255.255"
    finally:
        s.close()

    # Step 2: parse `ifconfig` output to find the interface owning that IP
    # and read its broadcast field. macOS prints a line per inet:
    #   inet 192.168.76.42 netmask 0xfffffe00 broadcast 192.168.77.255
    try:
        out = subprocess.check_output(["ifconfig"], text=True, timeout=2)
    except (subprocess.SubprocessError, FileNotFoundError, OSError):
        return "255.255.255.255"

    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 2 or parts[0] != "inet":
            continue
        if parts[1] != local_ip:
            continue
        # Found our interface line; pull the broadcast field if present.
        try:
            bcast_idx = parts.index("broadcast") + 1
            return parts[bcast_idx]
        except (ValueError, IndexError):
            return "255.255.255.255"

    return "255.255.255.255"


try:
    from pygeomag import GeoMag as _PyGeoMag
    _GM = _PyGeoMag(coefficients_file="wmm/WMM_2025.COF")
except ImportError:
    _GM = None


def _approx_mag_declination(lat: float, lng: float) -> float:
    """Magnetic declination in degrees (east positive) at the given point.

    Uses pygeomag (NOAA WMM-2025 reference implementation) when
    installed — same model and coefficients tachbase ships in its
    precomputed grid. Result is accurate to ~0.1° anywhere on Earth.

    Falls back to a one-line linear approximation if pygeomag isn't
    installed, just to keep the sim runnable without setup. The
    approximation produces a permanent ~3° crab in the displayed track
    diamond (versus pygeomag's near-zero), so installing pygeomag is
    the right move for any real testing:

        pip install pygeomag
    """
    if _GM is None:
        return -0.5 * lng - 45.0
    # pygeomag refuses lat ±90 exactly — nudge the input so a sim flying
    # across the geographic pole doesn't crash.
    eval_lat = max(-89.999, min(89.999, lat))
    # Decimal year (UTC). WMM-2025 is valid 2025.0–2030.0; pygeomag will
    # warn if we extrapolate past that.
    import datetime
    now = datetime.datetime.now(datetime.timezone.utc)
    year_start = datetime.datetime(now.year, 1, 1, tzinfo=datetime.timezone.utc)
    next_year = datetime.datetime(now.year + 1, 1, 1, tzinfo=datetime.timezone.utc)
    decimal_year = now.year + (now - year_start).total_seconds() / (next_year - year_start).total_seconds()
    return _GM.calculate(glat=eval_lat, glon=lng, alt=0, time=decimal_year).d


def _derive_ahrs(hdg_deg: float, gs_kt: float, vs_fpm: float,
                 prev_hdg: float | None, prev_t: float, now: float) -> tuple[float, float, float]:
    """Fake AHRS from flight-path state. Real Stratus sends measured attitude;
    we're driving a G1000 PFD with a physics-free sim so we back into it from
    heading rate (→ bank) and vertical speed (→ pitch). Good enough to verify
    the PFD plumbing end-to-end. Returns (roll, pitch, heading) in degrees."""
    # Pitch from flight-path angle. gs_kt * 101.27 ≈ gs in ft/min, so the
    # ratio is dimensionless. atan gives the climb/descent angle.
    pitch = 0.0
    if gs_kt > 5.0:
        pitch = math.degrees(math.atan2(vs_fpm, gs_kt * 101.27))

    # Bank from coordinated-turn formula: tan(φ) = V · ω / g.
    # V in ft/s, ω in rad/s, g = 32.174 ft/s². Fall back to 0 on the first
    # sample or after a long gap (no previous heading to diff against).
    roll = 0.0
    dt = now - prev_t
    if prev_hdg is not None and 0.05 <= dt <= 2.0 and gs_kt > 15.0:
        # Normalize heading delta to [-180, +180] so a wrap from 359 → 001
        # doesn't read as a -358°/sec turn.
        d = ((hdg_deg - prev_hdg) + 540.0) % 360.0 - 180.0
        turn_rate_rad = math.radians(d / dt)
        v_fps = gs_kt * 1.68781
        tan_phi = (v_fps * turn_rate_rad) / 32.174
        # Clamp to ±60° — a real aircraft pushed past that is in an upset
        # and the standard formula stops being coordinated anyway.
        roll = max(-60.0, min(60.0, math.degrees(math.atan(tan_phi))))

    return roll, pitch, hdg_deg


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
# SO_BROADCAST is required to send to a broadcast address; without it the
# kernel returns EACCES on sendto. The Stratus listener on the iPad
# (GDL90Service.swift) already binds INADDR_ANY with SO_BROADCAST set, so
# broadcast packets land the same as unicast.
_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


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
            now = time.time()

            # The pilot enters a magnetic heading in the sim UI. To match
            # what real Stratus hardware emits, we have to send the two
            # GDL90 messages in their respective reference frames:
            #   - 0x0A Ownship Report → TRUE track (the spec's track-type
            #     bit is set to "true track" by gdl90._report)
            #   - 0x65 ForeFlight AHRS → MAGNETIC heading (per spec)
            # If we sent the same value for both, the receiver would
            # convert ownship-true to magnetic via local declination and
            # the result would disagree with the AHRS magnetic by exactly
            # the local variation — drawing a permanent ~14° crab in the
            # HSI track diamond even with zero wind. So convert the
            # input magnetic heading to true here for the GPS message,
            # using the same linear declination model the iPad uses.
            decl = _approx_mag_declination(lat, lon)
            track_true = (hdg + decl) % 360.0

            _send_udp(gdl90.ownship_report(lat, lon, alt, track_true, gs, vs, callsign=callsign))
            _send_udp(gdl90.ownship_geo_altitude(alt))

            # Fake AHRS derived from heading-rate + VS so the G1000-style PFD
            # has something to render. Updated in place so the next /pos sees
            # this sample as its "previous". hdg stays magnetic — both the
            # AHRS message and the heading-rate diff are in the same frame.
            roll, pitch, _ = _derive_ahrs(
                hdg, gs, vs,
                prev_hdg=_state["prev_hdg"],
                prev_t=_state["prev_t"],
                now=now,
            )
            _send_udp(gdl90.ahrs_foreflight(roll, pitch, hdg))
            _state["prev_hdg"] = hdg
            _state["prev_t"] = now
            _state["last_pos_at"] = now

            if not _state["heartbeat_thread_started"]:
                _state["heartbeat_thread_started"] = True
                t = threading.Thread(target=_heartbeat_loop, daemon=True)
                t.start()

            # Return current local declination so the browser physics can
            # interpret State.hdg as magnetic (matching what the iPad PFD
            # displays) and convert to true for the position step. Without
            # this the airplane physically moves at the *true* compass
            # bearing of the magnetic value, which slides the aircraft off
            # any GPS course at full local-variation drift rate.
            return self._send_json({"ok": True, "decl": decl})

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
    parser.add_argument("--target", default="auto",
                        help="UDP destination host:port. Default 'auto' broadcasts "
                             "on the Wi-Fi subnet (matches how a real Stratus "
                             "behaves). Use 127.0.0.1:4000 for iOS Simulator.")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    args = parser.parse_args()

    if args.target == "auto":
        target_host = _detect_subnet_broadcast()
        target_port = 4000
    else:
        host, _, port = args.target.partition(":")
        target_host = host or _detect_subnet_broadcast()
        target_port = int(port or "4000")
    _state["target"] = (target_host, target_port)

    server = ThreadingHTTPServer((args.host, args.port), SimHandler)
    print(f"[sim] http://{args.host}:{args.port}")
    print(f"[sim] UDP target {_state['target'][0]}:{_state['target'][1]}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
