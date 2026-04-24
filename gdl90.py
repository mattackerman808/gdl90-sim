"""GDL90 message framing and encoders.

Implements just enough of FAA GDL90 spec (FAA-E-2937/2) to let the sim
emit ownship + heartbeat + geometric altitude packets that Tachbase's
native GDL90Parser accepts. Pure stdlib, no external deps.

Wire format per message: 0x7E | msg_id | payload | CRC16 LSB | CRC16 MSB | 0x7E
with byte-stuffing applied to msg_id+payload+CRC: 0x7E -> 0x7D 0x5E,
0x7D -> 0x7D 0x5D.
"""

from __future__ import annotations

import struct
import time


_CRC_POLY = 0x1021


def _crc16_ccitt(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ _CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _byte_stuff(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b == 0x7E or b == 0x7D:
            out.append(0x7D)
            out.append(b ^ 0x20)
        else:
            out.append(b)
    return bytes(out)


def frame(msg_id: int, payload: bytes) -> bytes:
    """Wrap a message body in GDL90 framing with CRC."""
    body = bytes([msg_id]) + payload
    crc = _crc16_ccitt(body)
    body_with_crc = body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return b"\x7E" + _byte_stuff(body_with_crc) + b"\x7E"


def _encode_lat_lon_24(deg: float) -> bytes:
    """24-bit signed lat/lon, resolution 180/2^23 degrees per LSB."""
    raw = int(round(deg * (8388608.0 / 180.0)))
    if raw < 0:
        raw += 0x1000000
    raw &= 0xFFFFFF
    return bytes([(raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF])


def heartbeat(gps_valid: bool = True, utc_ok: bool = True, timestamp: int | None = None) -> bytes:
    """GDL90 Heartbeat (msg 0x00). 6-byte payload.

    Byte 0: status 1 (bit 7 = GPS position valid, bit 0 = UAT initialized)
    Byte 1: status 2 (bit 7 = time stamp MS bit, bit 0 = UTC OK)
    Bytes 2-3: timestamp seconds since 0000Z (LSB first; bit 16 lives in status 2 bit 7)
    Bytes 4-5: message counts (uplink + basic/long), set to zero
    """
    if timestamp is None:
        now = time.gmtime()
        timestamp = now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec

    status1 = 0x01  # UAT initialized
    if gps_valid:
        status1 |= 0x80

    ts_low = timestamp & 0xFFFF
    ts_high_bit = (timestamp >> 16) & 0x01

    status2 = 0
    if utc_ok:
        status2 |= 0x01
    if ts_high_bit:
        status2 |= 0x80

    payload = struct.pack("<BBHBB", status1, status2, ts_low, 0, 0)
    return frame(0x00, payload)


def _report(
    msg_id: int,
    lat: float,
    lon: float,
    alt_ft: float,
    track_deg: float,
    speed_kt: float,
    vspeed_fpm: float,
    *,
    address: int = 0xF00D00,
    callsign: str = "SIM",
    airborne: bool = True,
    emitter_category: int = 1,
) -> bytes:
    """Shared encoder for Ownship (0x0A) and Traffic (0x14) reports.

    27-byte payload. See GDL90 spec section 3.5.1 or GDL90Parser.swift for
    the exact bit packing, which we mirror here.
    """
    # Byte 0: alert status (4) | address type (4). 0x01 = self-assigned address.
    b0 = 0x01

    # Bytes 1-3: participant address (24-bit)
    addr = address & 0xFFFFFF
    addr_bytes = bytes([(addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF])

    lat_bytes = _encode_lat_lon_24(lat)
    lon_bytes = _encode_lat_lon_24(lon)

    # Altitude: (alt+1000)/25 into a 12-bit field, clamped
    alt_raw = int(round((alt_ft + 1000.0) / 25.0))
    alt_raw = max(0, min(0xFFF, alt_raw))
    # Byte 10: high 8 bits of 12-bit alt
    b10 = (alt_raw >> 4) & 0xFF
    # Byte 11: low 4 bits of alt | misc nibble (bit 3 = airborne, bit 0 = tt valid)
    misc = 0x09 if airborne else 0x01  # airborne + true-track heading type
    b11 = ((alt_raw & 0x0F) << 4) | (misc & 0x0F)

    # Byte 12: NIC (4) | NACp (4). 0x88 = NIC 8 / NACp 8 — typical for good GPS.
    b12 = 0x88

    # Horizontal velocity (kt, 12-bit). 0xFFF = unavailable, cap at 0xFFE.
    hv_raw = int(round(max(0.0, speed_kt)))
    hv_raw = max(0, min(0xFFE, hv_raw))

    # Vertical velocity (64 fpm/LSB, 12-bit signed). 0x800 = unavailable.
    vv_raw = int(round(vspeed_fpm / 64.0))
    if vv_raw > 0x1FD:  # limit per spec (+32576 fpm)
        vv_raw = 0x1FD
    if vv_raw < -0x1FD:
        vv_raw = -0x1FD
    if vv_raw < 0:
        vv_raw += 0x1000
    vv_raw &= 0xFFF

    # Byte 13: high 8 bits of horizontal velocity
    b13 = (hv_raw >> 4) & 0xFF
    # Byte 14: low 4 bits of h.vel | high 4 bits of v.vel
    b14 = ((hv_raw & 0x0F) << 4) | ((vv_raw >> 8) & 0x0F)
    # Byte 15: low 8 bits of v.vel
    b15 = vv_raw & 0xFF

    # Byte 16: track, 360/256 deg per LSB
    track_norm = track_deg % 360.0
    b16 = int(round(track_norm * (256.0 / 360.0))) & 0xFF

    # Byte 17: emitter category
    b17 = emitter_category & 0xFF

    # Bytes 18-25: callsign, 8 ASCII, space-padded
    cs = (callsign or "")[:8].upper()
    cs_bytes = cs.encode("ascii", errors="replace").ljust(8, b" ")

    # Byte 26: emergency priority (4) | spare (4). 0 = no emergency.
    b26 = 0x00

    payload = (
        bytes([b0]) + addr_bytes + lat_bytes + lon_bytes +
        bytes([b10, b11, b12, b13, b14, b15, b16, b17]) +
        cs_bytes + bytes([b26])
    )
    assert len(payload) == 27, len(payload)
    return frame(msg_id, payload)


def ownship_report(
    lat: float, lon: float, alt_ft: float,
    track_deg: float, speed_kt: float, vspeed_fpm: float,
    callsign: str = "SIM",
) -> bytes:
    return _report(0x0A, lat, lon, alt_ft, track_deg, speed_kt, vspeed_fpm, callsign=callsign)


def traffic_report(
    address: int, lat: float, lon: float, alt_ft: float,
    track_deg: float, speed_kt: float, vspeed_fpm: float,
    callsign: str = "", emitter_category: int = 1,
) -> bytes:
    return _report(
        0x14, lat, lon, alt_ft, track_deg, speed_kt, vspeed_fpm,
        address=address, callsign=callsign, emitter_category=emitter_category,
    )


def ownship_geo_altitude(alt_ft: float, vfom_m: int = 10) -> bytes:
    """GDL90 Ownship Geometric Altitude (msg 0x0B). 4-byte payload.

    Bytes 0-1: altitude in 5-ft increments (signed Int16).
    Bytes 2-3: VFOM in meters (bit 15 = warning flag, cleared).
    """
    alt_raw = int(round(alt_ft / 5.0))
    alt_raw = max(-32768, min(32767, alt_raw))
    if alt_raw < 0:
        alt_raw += 0x10000
    payload = struct.pack(">HH", alt_raw & 0xFFFF, vfom_m & 0x7FFF)
    return frame(0x0B, payload)


def ahrs_foreflight(roll_deg: float, pitch_deg: float, heading_deg: float) -> bytes:
    """ForeFlight AHRS (msg 0x65 subID 1). Matches GDL90Parser parseAHRS.

    All three fields are Int16 in tenths of a degree.
    """
    def i16_tenths(deg: float) -> int:
        v = int(round(deg * 10.0))
        v = max(-32768, min(32767, v))
        if v < 0:
            v += 0x10000
        return v & 0xFFFF

    payload = struct.pack(
        ">BHHH", 0x01,
        i16_tenths(roll_deg),
        i16_tenths(pitch_deg),
        i16_tenths(heading_deg),
    )
    return frame(0x65, payload)
