"""
test_fb.py — unit tests for the FlatBuffers racket protocol.

Tests the full encode → decode round-trip using the same byte layouts
produced by flatbuf.c on the firmware side.

Run:
    python test_fb.py
"""

import struct
from racket_fb import decode_imu, decode_hit, FrameReader, FB_TYPE_IMU, FB_TYPE_HIT


# ── minimal C-side builder reimplemented in Python for test purposes ───────────

def _w16(buf, off, v): struct.pack_into('<H', buf, off, v & 0xFFFF)
def _w32(buf, off, v): struct.pack_into('<I', buf, off, v & 0xFFFFFFFF)
def _wi16(buf, off, v): struct.pack_into('<h', buf, off, v)
def _wi32(buf, off, v): struct.pack_into('<i', buf, off, v)

def build_imu(timestamp_ms, yaw, roll, pitch, ax, ay, az) -> bytes:
    buf = bytearray(42)
    _w32(buf,  0, 22)
    _w16(buf,  4, 18); _w16(buf,  6, 20)
    _w16(buf,  8,  4); _w16(buf, 10,  8); _w16(buf, 12, 10)
    _w16(buf, 14, 12); _w16(buf, 16, 14); _w16(buf, 18, 16); _w16(buf, 20, 18)
    _wi32(buf, 22, -18)
    _w32(buf, 26, timestamp_ms)
    _wi16(buf, 30, yaw); _wi16(buf, 32, roll); _wi16(buf, 34, pitch)
    _wi16(buf, 36, ax);  _wi16(buf, 38, ay);   _wi16(buf, 40, az)
    return bytes(buf)

def build_hit(timestamp_ms, region, peak_raw, heatmap) -> bytes:
    buf = bytearray(54)
    _w32(buf,  0, 16)
    _w16(buf,  4, 12); _w16(buf,  6, 16)
    _w16(buf,  8,  8); _w16(buf, 10, 14); _w16(buf, 12, 12); _w16(buf, 14, 4)
    _wi32(buf, 16, -12)
    _w32(buf, 20, 12)
    _w32(buf, 24, timestamp_ms)
    _w16(buf, 28, peak_raw)
    buf[30] = region; buf[31] = 0
    _w32(buf, 32, 9)
    for i, v in enumerate(heatmap):
        _w16(buf, 36 + i * 2, v)
    return bytes(buf)

def make_frame(type_tag, payload) -> bytes:
    length = len(payload)
    return bytes([type_tag, length & 0xFF, length >> 8]) + payload


# ── tests ──────────────────────────────────────────────────────────────────────

def test_imu_decode():
    payload = build_imu(12500, 33, -7, 12, 30, -21, 985)
    assert len(payload) == 42, f"expected 42 bytes, got {len(payload)}"
    d = decode_imu(payload)
    assert d["timestamp_ms"] == 12500
    assert d["yaw_deg"]      == 33
    assert d["roll_deg"]     == -7
    assert d["pitch_deg"]    == 12
    assert d["ax_mg"]        == 30
    assert d["ay_mg"]        == -21
    assert d["az_mg"]        == 985
    print("PASS  test_imu_decode")

def test_hit_decode():
    hm = [0, 1, 0, 0, 5, 0, 3, 0, 0]
    payload = build_hit(5000, 4, 2100, hm)
    assert len(payload) == 54, f"expected 54 bytes, got {len(payload)}"
    d = decode_hit(payload)
    assert d["timestamp_ms"] == 5000
    assert d["region"]       == 4
    assert d["peak_raw"]     == 2100
    assert d["heatmap"]      == hm
    print("PASS  test_hit_decode")

def test_imu_negative_values():
    payload = build_imu(0, -180, -90, -45, -1000, -500, 0)
    d = decode_imu(payload)
    assert d["yaw_deg"]   == -180
    assert d["roll_deg"]  == -90
    assert d["pitch_deg"] == -45
    assert d["ax_mg"]     == -1000
    assert d["ay_mg"]     == -500
    assert d["az_mg"]     == 0
    print("PASS  test_imu_negative_values")

def test_frame_reader():
    """Simulate FrameReader over a byte stream with garbage prefix."""
    import io

    imu_payload = build_imu(9999, 10, 5, -3, 100, 200, 980)
    hit_hm      = [2, 0, 0, 1, 3, 0, 0, 0, 1]
    hit_payload = build_hit(1000, 2, 750, hit_hm)

    # Stream: garbage byte + IMU frame + HIT frame
    stream = bytes([0xAA]) + make_frame(FB_TYPE_IMU, imu_payload) \
                           + make_frame(FB_TYPE_HIT, hit_payload)

    class FakeSerial:
        """Minimal serial.Serial mock that reads from a bytes buffer."""
        def __init__(self, data):
            self._buf = io.BytesIO(data)
        def read(self, n):
            return self._buf.read(n)

    reader = FrameReader(FakeSerial(stream))
    frames = reader.read_frames(timeout_s=9999)  # timeout_s ignored by FakeSerial

    assert len(frames) == 2, f"expected 2 frames, got {len(frames)}"

    t0, p0 = frames[0]
    assert t0 == FB_TYPE_IMU
    d0 = decode_imu(p0)
    assert d0["timestamp_ms"] == 9999
    assert d0["yaw_deg"]      == 10

    t1, p1 = frames[1]
    assert t1 == FB_TYPE_HIT
    d1 = decode_hit(p1)
    assert d1["region"]  == 2
    assert d1["heatmap"] == hit_hm

    print("PASS  test_frame_reader  (garbage-prefix resync + 2 frames)")


if __name__ == "__main__":
    test_imu_decode()
    test_hit_decode()
    test_imu_negative_values()
    test_frame_reader()
    print("\nAll tests passed.")
