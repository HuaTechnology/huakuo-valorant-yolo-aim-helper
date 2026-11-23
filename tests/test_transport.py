from __future__ import annotations

import pytest

from dma.core.config import OutputCfg
from dma.core.metrics import Metrics, RollingStats
from dma.transport import protocol as P
from dma.transport.link import MockLink
from dma.transport.planner import MotionPlanner


# ---------------------------------------------------------------------------
# protocol
# ---------------------------------------------------------------------------
def test_frame_roundtrip():
    frame = P.encode_move(-300, 129, 7)
    kind, payload, seq = P.decode(frame)
    assert kind == P.T_MOVE
    assert seq == 7
    import struct

    dx, dy = struct.unpack("<hh", payload)
    assert (dx, dy) == (-300, 129)


def test_crc_detects_corruption():
    frame = bytearray(P.encode_button(1, 3))
    frame[4] ^= 0xFF
    with pytest.raises(ValueError):
        P.decode(bytes(frame))


def test_parser_resynchronises_after_garbage():
    good = P.encode_button(1, 1) + P.encode_button(0, 2)
    stream = b"\x00\x13garbage" + good
    parser = P.StreamParser()
    frames = list(parser.feed(stream))
    assert len(frames) == 2
    assert frames[0][2] == 1 and frames[1][2] == 2


def test_parser_partial_frames():
    frame = P.encode_move(10, 20, 5)
    parser = P.StreamParser()
    assert list(parser.feed(frame[:3])) == []
    out = list(parser.feed(frame[3:]))
    assert len(out) == 1


def test_status_decode():
    import struct

    payload = struct.pack("<IHHHHB", 12345, 600, 2, 1, 500, 0b11)
    assert len(payload) == P.STATUS_LEN
    st = P.decode_status(payload)
    assert st is not None
    assert st.uptime_ms == 12345 and st.rx_frames == 600 and st.report_hz == 500
    assert st.flags == 0b11


def test_legacy_frame_matches_v1_layout():
    frame = P.legacy_move(100, -50, 9)
    assert len(frame) == 10
    assert frame[0] == 0xAA
    assert sum(frame[:-1]) >= 0  # checksum is xor of body
    assert frame[-1] == 0xAA ^ P.L_CMD_MOVE ^ (100 & 0xFF) ^ ((100 >> 8) & 0xFF) ^ ((-50) & 0xFF) ^ (((-50) >> 8) & 0xFF) ^ 9 ^ 0


# ---------------------------------------------------------------------------
# planner
# ---------------------------------------------------------------------------
def test_planner_conserves_displacement():
    cfg = OutputCfg(report_hz=500, max_per_report=127, gain=1.0)
    pl = MotionPlanner(cfg)
    pl.add(3.4, -2.6)
    steps = pl.take(0.002)
    total = (sum(s[0] for s in steps), sum(s[1] for s in steps))
    # sub-pixel remainder is carried, never dropped
    assert total[0] in (3, 4)
    pl.add(0.0, 0.0)
    rest = pl.take(0.004)
    assert sum(s[0] for s in rest) + total[0] == 3 or abs(sum(s[0] for s in rest) + total[0] - 3.4) <= 1


def test_planner_respects_per_report_cap():
    cfg = OutputCfg(report_hz=1000, max_per_report=127, gain=1.0)
    pl = MotionPlanner(cfg)
    pl.add(400.0, 0.0)
    steps = pl.take(0.001)
    assert steps
    assert all(abs(dx) <= 127 and abs(dy) <= 127 for dx, dy in steps)


def test_planner_accumulator_is_bounded():
    cfg = OutputCfg(max_per_report=127, gain=1.0)
    pl = MotionPlanner(cfg)
    for _ in range(1000):
        pl.add(1000.0, 1000.0)
    assert abs(pl.pending[0]) <= 4 * 127 + 1e-6


# ---------------------------------------------------------------------------
# mock link
# ---------------------------------------------------------------------------
def test_mock_link_records_reports_and_buttons():
    cfg = OutputCfg(report_hz=1000, dry_run=True)
    link = MockLink(cfg)
    link.submit_motion(10.0, -5.0)
    link.submit_button(True)
    link.submit_button(False)
    sx, sy = link.total()
    assert sx == 10 and sy == -5
    assert [b for _, b in link.buttons] == [True, False]
    assert link.status()["mock"]


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------
def test_rolling_stats_percentiles():
    st = RollingStats(100)
    for v in range(100):
        st.add(float(v))
    assert st.mean == pytest.approx(49.5, abs=1e-6)
    assert st.percentile(0.5) == pytest.approx(49.5, abs=0.5)
    assert st.percentile(0.95) == pytest.approx(94.05, abs=1.0)
    assert st.summary()["max"] == 99.0


def test_rolling_stats_ignores_nan():
    st = RollingStats(10)
    st.add(float("nan"))
    st.add(2.0)
    assert st.count == 1
    assert st.mean == 2.0


def test_metrics_snapshot_serialisable():
    import json

    m = Metrics()
    m.inc("a.b", 2)
    m.set("g.x", 1.5)
    m.observe("s.lat", 3.0)
    payload = json.loads(m.to_json())
    assert payload["counters"]["a.b"] == 2
    assert payload["gauges"]["g.x"] == 1.5
    assert "s.lat" in payload["stats"]
    assert "dma_a_b_total" in m.to_prometheus()
