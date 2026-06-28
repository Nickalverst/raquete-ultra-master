"""
racket_fb.py — FlatBuffers decoder for the instrumented racket.

Wire framing:
    [type : uint8][length : uint16 LE] → then `length` bytes of FlatBuffer
    type 0x01 = ImuPacket  (always 42 bytes)
    type 0x02 = HitPacket  (always 54 bytes)

No external flatbuffers library required — decoding is done with struct
using the same fixed layout the firmware builds in flatbuf.c.

Public API:
    FB_TYPE_IMU, FB_TYPE_HIT   — frame type constants
    decode_imu(buf)            — bytes → dict
    decode_hit(buf)            — bytes → dict
    FrameReader(ser)           — reads framed packets from a serial.Serial
"""

from __future__ import annotations

import struct
import time
from typing import Optional

# ── type tags (must match flatbuf.h) ──────────────────────────────────────────
FB_TYPE_IMU: int = 0x01
FB_TYPE_HIT: int = 0x02

_KNOWN_TYPES  = {FB_TYPE_IMU, FB_TYPE_HIT}
_KNOWN_SIZES  = {FB_TYPE_IMU: 42, FB_TYPE_HIT: 54}


# ── generic FlatBuffers table accessor ────────────────────────────────────────

def _make_accessors(buf: bytes):
    """Return field-accessor closures for a FlatBuffer rooted at byte 0."""
    table_pos = struct.unpack_from('<I', buf, 0)[0]
    vtable_soffset = struct.unpack_from('<i', buf, table_pos)[0]
    vtable_pos = table_pos + vtable_soffset
    vtable_size = struct.unpack_from('<H', buf, vtable_pos)[0]

    def voff(fid: int) -> int:
        p = vtable_pos + 4 + fid * 2
        if p + 2 > vtable_pos + vtable_size:
            return 0
        return struct.unpack_from('<H', buf, p)[0]

    def u32(fid: int) -> int:
        o = voff(fid)
        return struct.unpack_from('<I', buf, table_pos + o)[0] if o else 0

    def i16(fid: int) -> int:
        o = voff(fid)
        return struct.unpack_from('<h', buf, table_pos + o)[0] if o else 0

    def u16(fid: int) -> int:
        o = voff(fid)
        return struct.unpack_from('<H', buf, table_pos + o)[0] if o else 0

    def u8(fid: int) -> int:
        o = voff(fid)
        return buf[table_pos + o] if o else 0

    def vec_u16(fid: int) -> list[int]:
        o = voff(fid)
        if not o:
            return []
        uoff_pos = table_pos + o
        uoff = struct.unpack_from('<I', buf, uoff_pos)[0]
        vec_pos = uoff_pos + uoff
        count = struct.unpack_from('<I', buf, vec_pos)[0]
        return [struct.unpack_from('<H', buf, vec_pos + 4 + i * 2)[0]
                for i in range(count)]

    return u32, i16, u16, u8, vec_u16


# ── public decoders ───────────────────────────────────────────────────────────

def decode_imu(buf: bytes) -> dict:
    """
    Decode an ImuPacket FlatBuffer (42 bytes).

    Returns:
        {timestamp_ms, yaw_deg, roll_deg, pitch_deg, ax_mg, ay_mg, az_mg}
    """
    u32, i16, *_ = _make_accessors(buf)
    return {
        "timestamp_ms": u32(0),
        "yaw_deg":      i16(1),
        "roll_deg":     i16(2),
        "pitch_deg":    i16(3),
        "ax_mg":        i16(4),
        "ay_mg":        i16(5),
        "az_mg":        i16(6),
    }


def decode_hit(buf: bytes) -> dict:
    """
    Decode a HitPacket FlatBuffer (54 bytes).

    Returns:
        {timestamp_ms, region, peak_raw, heatmap}
        heatmap is a list[int] of length 9 (row-major 3×3 grid).
    """
    u32, _i16, u16, u8, vec_u16 = _make_accessors(buf)
    heatmap = vec_u16(3)
    if len(heatmap) < 9:
        heatmap += [0] * (9 - len(heatmap))
    return {
        "timestamp_ms": u32(0),
        "region":       u8(1),
        "peak_raw":     u16(2),
        "heatmap":      heatmap,
    }


# ── framing reader ────────────────────────────────────────────────────────────

class FrameReader:
    """
    Reads binary frames from a pyserial Serial object.

    Each frame:  [type:1][len_lo:1][len_hi:1][flatbuffer:len]

    Bytes that are not a recognised type tag are silently discarded,
    giving the reader automatic re-sync after garbage (e.g. boot printf
    output from the firmware before the scheduler starts).
    """

    def __init__(self, ser) -> None:
        self._ser = ser

    def read_frames(self, timeout_s: float = 0.08) -> list[tuple[int, bytes]]:
        """
        Read all complete frames available within *timeout_s* seconds.

        Returns a list of (type, payload_bytes) tuples.
        """
        frames: list[tuple[int, bytes]] = []
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            # ── type byte ──────────────────────────────────────────────────
            raw = self._ser.read(1)
            if not raw:
                break                        # serial timeout — no more data
            t = raw[0]
            if t not in _KNOWN_TYPES:
                continue                     # skip / re-sync

            # ── 2-byte length ──────────────────────────────────────────────
            raw = self._ser.read(2)
            if len(raw) < 2:
                break
            length = struct.unpack_from('<H', raw)[0]

            # Sanity: length must match the expected fixed size
            if length != _KNOWN_SIZES[t]:
                continue

            # ── payload ────────────────────────────────────────────────────
            payload = self._ser.read(length)
            if len(payload) < length:
                break                        # truncated frame — stop here

            frames.append((t, bytes(payload)))

        return frames
