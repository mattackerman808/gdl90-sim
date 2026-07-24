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
import urllib.parse
import urllib.request
from datetime import date, timedelta
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
    # List of (host, port) UDP destinations. Plural so the auto path can
    # fan out to v4 subnet broadcast + v6 link-local all-nodes multicast at
    # once. Explicit --target / UI overrides collapse this back to a single
    # entry. Replaced at startup before any packet goes out.
    "targets": [("255.255.255.255", 4000)],
    "last_pos_at": 0.0,
    "heartbeat_thread_started": False,
    # Previous /pos sample used to derive bank angle from turn rate.
    "prev_hdg": None,
    "prev_t": 0.0,
    # Last known ownship state, used as the anchor when a scenario spawns
    # a relative target ("3 nm at 12 o'clock"). Updated on every /pos.
    "ownship": None,        # {"lat","lon","alt","hdg","gs","vs","t"}
    # Active traffic targets keyed by hex id. Each entry is a dict with
    # current state + the fixed velocity vector. Ticked forward on every
    # /pos so each ownship update gets a matching traffic_report frame.
    "traffic": {},          # hex_int → state dict
    "traffic_lock": threading.Lock(),
    "traffic_t": 0.0,
}


# Conservative cleanup: drop targets that drift more than 25 nm from
# ownship — they're not threats, and leaving them queued blows the
# trafficCache on the iPad pointlessly.
TRAFFIC_DROP_RANGE_NM = 25.0
EARTH_NM = 3440.065


def _great_circle_nm(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return EARTH_NM * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _project(lat, lon, track_deg, dist_nm):
    """Move (lat, lon) `dist_nm` along bearing `track_deg`. Flat-earth
    approximation — fine for the few-mile distances we project per tick."""
    cos_lat = max(0.01, math.cos(math.radians(lat)))
    rad = math.radians(track_deg)
    dlat = (dist_nm * math.cos(rad)) / 60.0
    dlon = (dist_nm * math.sin(rad)) / (60.0 * cos_lat)
    return lat + dlat, lon + dlon


def _spawn_relative_target(scenario):
    """Build a target dict from a scenario spec relative to current ownship.

    scenario keys (all optional except clock + range_nm):
      hex            24-bit address (auto-assigned if omitted)
      callsign       up to 8 chars (default 'SIM' + sequence)
      clock          1..12, where target lat/lon is placed at this clock
                     position relative to ownship's current heading
      range_nm       distance from ownship at spawn time
      alt_diff_ft    target altitude minus ownship altitude
      track          target track (true deg). Common patterns:
                       "head_on"   → ownship_track + 180
                       "parallel"  → ownship_track
                       "crossing"  → ownship_track + 90
                       <number>    → absolute track in degrees
      gs_kt          target ground speed (default 150)
      vspeed_fpm     target vertical speed (default 0)
      emitter        emitter category (default 1, light fixed-wing)
    """
    own = _state["ownship"]
    if not own:
        return None, "no ownship — fly a moment first so the spawn point is real"

    clock = int(scenario.get("clock", 12))
    if clock < 1 or clock > 12:
        return None, "clock must be 1..12"
    range_nm = float(scenario.get("range_nm", 3.0))
    alt_diff = float(scenario.get("alt_diff_ft", 0.0))

    # Place at the clock position relative to ownship's current track.
    rel_bearing = ((clock % 12) * 30.0)  # 12 → 0, 3 → 90, 6 → 180, 9 → 270
    spawn_track = (own["hdg"] + rel_bearing) % 360.0
    lat, lon = _project(own["lat"], own["lon"], spawn_track, range_nm)
    alt_ft = own["alt"] + alt_diff

    # Target track relative to ownship. The geometry that produces an
    # actual TA depends on how the target is moving relative to the
    # line-of-sight, not just its heading — so e.g. for a target at
    # 3 o'clock to *cross* our path (toward our nose) it has to head
    # toward our 9 o'clock, which is ownship_hdg - 90, not + 90.
    track_in = scenario.get("track", "head_on")
    if track_in == "head_on":
        target_track = (own["hdg"] + 180.0) % 360.0
    elif track_in == "parallel":
        target_track = own["hdg"]
    elif track_in in ("cross_left", "crossing"):
        # Target crosses leftward in front of us — used for a target at
        # the 1-5 o'clock side that needs to converge with our path.
        target_track = (own["hdg"] - 90.0 + 360.0) % 360.0
    elif track_in == "cross_right":
        # Mirror — for a target at 7-11 o'clock crossing rightward.
        target_track = (own["hdg"] + 90.0) % 360.0
    else:
        target_track = float(track_in) % 360.0

    gs_kt = float(scenario.get("gs_kt", 150.0))
    vspeed_fpm = float(scenario.get("vspeed_fpm", 0.0))
    emitter = int(scenario.get("emitter", 1))

    # Auto-assign hex if not provided. Random in 24-bit ICAO space.
    if "hex" in scenario:
        hex_id = int(scenario["hex"]) & 0xFFFFFF
    else:
        hex_id = (0xA00000 | (int(time.time() * 1000) & 0xFFFFFF)) & 0xFFFFFF

    # Default callsign: a plausible US registration derived from the hex,
    # not "SIM###" — ambient targets get tapped during screenshot sessions
    # and a synthetic-looking callsign in the detail card is a tell. Real
    # US N-numbers are N1..N99999 optionally ending in 1-2 letters (no
    # I/O); this yields e.g. N4213K deterministically per target.
    suffix_letters = "ABCDEFGHJKLMNPQRSTUVWXYZ"  # no I/O, per FAA rules
    default_cs = (f"N{(hex_id % 8999) + 1000}"
                  f"{suffix_letters[hex_id % len(suffix_letters)]}")
    callsign = str(scenario.get("callsign", default_cs))[:8]

    return {
        "hex": hex_id,
        "callsign": callsign,
        "lat": lat, "lon": lon, "alt": alt_ft,
        "track": target_track, "gs": gs_kt, "vspeed": vspeed_fpm,
        "emitter": emitter,
        "spawned_at": time.time(),
    }, None


def _advance_traffic(now):
    """Move every active target forward by elapsed time × velocity.
    Drops targets that have drifted past TRAFFIC_DROP_RANGE_NM."""
    if not _state["ownship"]:
        return
    last = _state["traffic_t"] or now
    dt = max(0.0, min(now - last, 5.0))     # clamp to 5s in case of stalls
    _state["traffic_t"] = now
    own_lat = _state["ownship"]["lat"]
    own_lon = _state["ownship"]["lon"]
    with _state["traffic_lock"]:
        drop = []
        for hex_id, t in _state["traffic"].items():
            dist = (t["gs"] / 3600.0) * dt    # nm in dt seconds
            t["lat"], t["lon"] = _project(t["lat"], t["lon"], t["track"], dist)
            t["alt"] += (t["vspeed"] / 60.0) * dt
            if _great_circle_nm(own_lat, own_lon, t["lat"], t["lon"]) > TRAFFIC_DROP_RANGE_NM:
                drop.append(hex_id)
        for hex_id in drop:
            del _state["traffic"][hex_id]


def _emit_traffic_frames():
    """Send one Traffic Report (msg 0x14) per active target. Called after
    the ownship frame on every /pos so the iPad's 1Hz cache flush sees a
    consistent snapshot."""
    with _state["traffic_lock"]:
        for t in _state["traffic"].values():
            try:
                _send_udp(gdl90.traffic_report(
                    address=t["hex"],
                    lat=t["lat"], lon=t["lon"], alt_ft=t["alt"],
                    track_deg=t["track"], speed_kt=t["gs"], vspeed_fpm=t["vspeed"],
                    callsign=t["callsign"], emitter_category=t["emitter"],
                ))
            except Exception as e:
                print(f"[sim] traffic_report failed for {t['hex']:06X}: {e}", file=sys.stderr)


def _detect_auto_targets(port: int = 4000) -> list[tuple[str, int]]:
    """Return UDP destinations for the auto-broadcast path: the v4 subnet
    broadcast AND the v6 link-local all-nodes multicast (ff02::1) on
    EVERY active LAN interface. Ethernet and Wi-Fi being up at the same
    time both get packets so an iPad on either link sees the sim.

    Mirrors what a Stratus does on its own AP: subnet broadcast on UDP 4000.
    Reads each interface's broadcast address from `ifconfig` — corporate
    Wi-Fi commonly uses /23 or /22, so guessing /24 sends to the wrong
    address. ff02::1 reaches every v6 host on the link (they all join by
    default), so it's the v6 equivalent of v4 broadcast.

    Filter: interface must carry BROADCAST + RUNNING flags AND have a v4
    address with a broadcast field. That naturally excludes loopback
    (no BROADCAST), down interfaces, VPN tunnels (POINTOPOINT, no
    BROADCAST), and Apple's AirDrop/awdl interfaces (v6-only). Docker /
    VirtualBox bridges pass the filter, which is harmless — packets just
    fall on the floor if no listener is on that bridge.

    Falls back to v4 limited broadcast (255.255.255.255) if nothing
    qualifies; the v6 leg is dropped in that case (we wouldn't know which
    interface to scope ff02::1 to).
    """
    try:
        out = subprocess.check_output(["ifconfig"], text=True, timeout=2)
    except (subprocess.SubprocessError, FileNotFoundError, OSError):
        return [("255.255.255.255", port)]

    # Parse ifconfig into per-interface blocks. A new block starts on any
    # line that begins flush-left (e.g. "en0: flags=...mtu 1500"); body
    # lines are indented.
    blocks: list[tuple[str, str, list[str]]] = []  # (name, header line, body)
    name = header = ""
    body: list[str] = []
    for line in out.splitlines():
        if line and not line[0].isspace():
            if name:
                blocks.append((name, header, body))
            name = line.split(":", 1)[0]
            header = line
            body = []
        else:
            body.append(line)
    if name:
        blocks.append((name, header, body))

    targets: list[tuple[str, int]] = []
    seen_v4: set[str] = set()
    for iface, hdr, lines in blocks:
        if "BROADCAST" not in hdr or "RUNNING" not in hdr:
            continue
        v4_addr = v4_bcast = None
        for line in lines:
            parts = line.split()
            if len(parts) >= 2 and parts[0] == "inet" and "broadcast" in parts:
                try:
                    v4_addr = parts[1]
                    v4_bcast = parts[parts.index("broadcast") + 1]
                    break
                except (ValueError, IndexError):
                    pass
        if not v4_bcast:
            continue
        # Drop /32 pseudo-broadcasts where the "broadcast" address equals
        # the host's own IP (Apple's 192.0.0.0/29 Wake-on-Wireless virtuals
        # do this). Sending there just bounces back to the local stack.
        if v4_bcast == v4_addr:
            continue
        if v4_bcast not in seen_v4:
            targets.append((v4_bcast, port))
            seen_v4.add(v4_bcast)
        targets.append((f"ff02::1%{iface}", port))

    return targets or [("255.255.255.255", port)]


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


# AIRAC anchor: cycle 2606 is effective 2026-06-11 (per
# tachbase-api lib/cycles.py / live /api/tile_manifest). Cycles are 28 days.
# Used as fallback when the live manifest can't be reached so we never ship
# a stale literal like the old "2604" default.
_AIRAC_ANCHOR_DATE = date(2026, 6, 11)


def _airac_cycle_for(today: date) -> str:
    """Compute the AIRAC cycle ID effective on `today` (YYCC, 1-indexed within
    the calendar year of the cycle's effective date)."""
    cycles_from_anchor = (today - _AIRAC_ANCHOR_DATE).days // 28
    cycle_eff = _AIRAC_ANCHOR_DATE + timedelta(days=cycles_from_anchor * 28)
    first_of_year = cycle_eff
    while (first_of_year - timedelta(days=28)).year == cycle_eff.year:
        first_of_year -= timedelta(days=28)
    cc = (cycle_eff - first_of_year).days // 28 + 1
    return f"{cycle_eff.year % 100:02d}{cc:02d}"


def _fetch_vfr_cycle() -> str:
    """Ask the live tachbase API for the current VFR tile cycle; fall back to
    the date-derived AIRAC cycle if the API is unreachable."""
    try:
        req = urllib.request.Request(
            "https://www.tachbase.com/api/tile_manifest",
            headers={"User-Agent": "gdl90-sim"},
        )
        with urllib.request.urlopen(req, timeout=4) as r:
            data = json.loads(r.read().decode("utf-8"))
            cycle = data.get("cycles", {}).get("vfr", {}).get("cycle")
            if cycle:
                return str(cycle)
    except Exception as e:
        print(f"[sim] manifest fetch failed ({e}); using computed AIRAC fallback", file=sys.stderr)
    return _airac_cycle_for(date.today())


# Cached cycle + last-fetch time. Refreshed on /config when older than the TTL
# below so a long-running sim picks up an AIRAC rotation without restart.
_VFR_CYCLE: str | None = None
_VFR_CYCLE_FETCHED_AT: float = 0.0
_VFR_CYCLE_TTL_SEC = 3600.0  # 1 hour


def _parse_target(s: str) -> tuple[str, int]:
    """Parse a `host:port` / `[v6]:port` / `host` target string. Defaults
    port to 4000. urlsplit understands the bracketed v6 form; the manual
    rsplit handles unbracketed v4 host:port."""
    parsed = urllib.parse.urlsplit("//" + s)
    if parsed.hostname is not None:
        return parsed.hostname, parsed.port or 4000
    host, _, port = s.rpartition(":")
    return (host or s, int(port) if port else 4000)


def _format_target(host: str, port: int) -> str:
    """Render host:port for display, bracketing v6 literals so the
    string round-trips through _parse_target."""
    return f"[{host}]:{port}" if ":" in host else f"{host}:{port}"


# UDP sockets keyed by address family. We need separate sockets for v4 vs v6
# (a v4 socket can't sendto a v6 address and vice versa). Created lazily so
# starting the sim doesn't open a socket family we never use.
_socks: dict[int, socket.socket] = {}
_socks_lock = threading.Lock()


def _get_sock(family: int) -> socket.socket:
    with _socks_lock:
        s = _socks.get(family)
        if s is not None:
            return s
        s = socket.socket(family, socket.SOCK_DGRAM)
        if family == socket.AF_INET:
            # SO_BROADCAST is required to send to a v4 broadcast address;
            # without it the kernel returns EACCES on sendto. Cheap to set
            # on the unicast path too. v6 has no broadcast — multicast
            # works without any extra socket option.
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        _socks[family] = s
        return s


def _send_udp(pkt: bytes) -> None:
    for host, port in _state["targets"]:
        try:
            # getaddrinfo picks the right family and fills in scope_id for
            # link-local v6 hosts (e.g. "fe80::1%en0" or "ff02::1%en0"),
            # which sendto needs to know which interface to egress on.
            family, _socktype, _proto, _canon, sockaddr = socket.getaddrinfo(
                host, port, type=socket.SOCK_DGRAM
            )[0]
            _get_sock(family).sendto(pkt, sockaddr)
        except (OSError, socket.gaierror) as e:
            print(f"[sim] UDP send failed to {_format_target(host, port)}: {e}", file=sys.stderr)


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
        if path == "/traffic":
            with _state["traffic_lock"]:
                out = [{
                    "hex": f"{t['hex']:06X}",
                    "callsign": t["callsign"],
                    "lat": t["lat"], "lon": t["lon"], "alt": int(t["alt"]),
                    "track": int(t["track"]), "gs": int(t["gs"]),
                    "vspeed": int(t["vspeed"]),
                    "spawned_at": int(t["spawned_at"]),
                } for t in _state["traffic"].values()]
            return self._send_json({"targets": out, "count": len(out)})

        if path == "/config":
            global _VFR_CYCLE, _VFR_CYCLE_FETCHED_AT
            if _VFR_CYCLE is None or (time.time() - _VFR_CYCLE_FETCHED_AT) > _VFR_CYCLE_TTL_SEC:
                _VFR_CYCLE = _fetch_vfr_cycle()
                _VFR_CYCLE_FETCHED_AT = time.time()
            targets = _state["targets"]
            return self._send_json({
                "vfr_cycle": _VFR_CYCLE,
                # First entry is the UI's display value (v4 in the auto
                # case). Full list exposed too so the user can see whether
                # the v6 multicast leg is also active.
                "target": _format_target(*targets[0]),
                "targets": [_format_target(h, p) for h, p in targets],
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

            # Snapshot ownship for the traffic-spawn anchor + advance and
            # emit any active traffic targets so the iPad's 1Hz cache
            # flush sees a fresh frame this tick.
            _state["ownship"] = {"lat": lat, "lon": lon, "alt": alt,
                                 "hdg": hdg, "gs": gs, "vs": vs, "t": now}
            _advance_traffic(now)
            _emit_traffic_frames()

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

        if path == "/traffic/spawn":
            target, err = _spawn_relative_target(data)
            if err:
                return self._send_json({"error": err}, status=400)
            with _state["traffic_lock"]:
                _state["traffic"][target["hex"]] = target
            return self._send_json({"ok": True, "hex": f"{target['hex']:06X}",
                                    "callsign": target["callsign"]})

        if path == "/traffic/clear":
            with _state["traffic_lock"]:
                count = len(_state["traffic"])
                _state["traffic"].clear()
            return self._send_json({"ok": True, "cleared": count})

        if path == "/target":
            try:
                host = str(data["host"])
                port = int(data["port"])
            except (KeyError, TypeError, ValueError):
                return self._send_json({"error": "need host/port"}, status=400)
            _state["targets"] = [(host, port)]
            print(f"[sim] UDP target -> {_format_target(host, port)}")
            return self._send_json({"ok": True, "target": _format_target(host, port)})

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
        _state["targets"] = _detect_auto_targets()
    else:
        target_host, target_port = _parse_target(args.target)
        if target_host:
            _state["targets"] = [(target_host, target_port)]
        else:
            # `--target :5000` (port override but no host) — keep the auto
            # fan-out and just rewrite the port on every leg.
            _state["targets"] = _detect_auto_targets(port=target_port)

    server = ThreadingHTTPServer((args.host, args.port), SimHandler)
    print(f"[sim] http://{args.host}:{args.port}")
    for h, p in _state["targets"]:
        print(f"[sim] UDP target {_format_target(h, p)}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
