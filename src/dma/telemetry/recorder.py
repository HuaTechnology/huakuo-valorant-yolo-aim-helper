"""Session recording: the raw material for replay, autotune and support bundles.

A v1 "debug log" was a CSV of three numbers plus a JPEG every two seconds.  That
cannot answer the only question that matters after a bad match — *was it the
tracker, the latency estimate, or the fire gate?* — because the intermediate
stages were never persisted.

The recorder writes one gzip JSONL stream with four record types:

    header   settings snapshot + model description (reproducibility)
    frame    capture ts, detections (source coords), infer time, camera sample
    tick     control ts, commanded displacement, selected target, latency estimate
    fire     trigger decisions and their reason

Detections are stored in *source* pixels together with the camera sample used to
build the world frame, so replay can reconstruct the enemy's world trajectory
exactly and then drive a different controller through the same scenario.
"""
from __future__ import annotations

import gzip
import json
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..core.config import Settings, to_dict
from ..core.types import Detection, DetectionFrame


class SessionRecorder:
    def __init__(self, directory: str | os.PathLike = "out/telemetry", settings: Settings | None = None,
                 model_info: Dict[str, Any] | None = None, enabled: bool = True):
        self.enabled = enabled
        self.dir = Path(directory)
        self._fh = None
        self.path: Optional[Path] = None
        self._lock = threading.Lock()
        self._n = 0
        if not enabled:
            return
        self.dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = self.dir / f"session_{stamp}.jsonl.gz"
        self._fh = gzip.open(self.path, "wt", encoding="utf-8", compresslevel=4)
        self._write({"type": "header", "ts": time.time(), "schema": 2,
                     "settings": to_dict(settings) if settings else None,
                     "model": model_info or {}})

    # ------------------------------------------------------------------
    def _write(self, record: Dict[str, Any]) -> None:
        if self._fh is None:
            return
        with self._lock:
            self._fh.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            self._n += 1

    def frame(self, det: DetectionFrame, cam: tuple[float, float]) -> None:
        if self._fh is None:
            return
        self._write(
            {
                "type": "frame",
                "ts": round(det.capture_ts, 6),
                "fid": det.frame_id,
                "ms": round(det.infer_ms, 3),
                "cam": [round(cam[0], 2), round(cam[1], 2)],
                "dets": [
                    [round(d.box.x1, 1), round(d.box.y1, 1), round(d.box.x2, 1), round(d.box.y2, 1),
                     d.cls_id, round(d.score, 3)]
                    for d in det.detections
                ],
            }
        )

    def tick(self, now: float, cam: tuple[float, float], u: tuple[float, float],
             target_id: Optional[int], err: tuple[float, float], mode: str,
             latency: Dict[str, float]) -> None:
        if self._fh is None:
            return
        self._write(
            {
                "type": "tick",
                "ts": round(now, 6),
                "cam": [round(cam[0], 2), round(cam[1], 2)],
                "u": [round(u[0], 2), round(u[1], 2)],
                "tid": target_id,
                "err": [round(err[0], 2), round(err[1], 2)],
                "mode": mode,
                "lat": {k: latency.get(k) for k in ("loop_ms", "confidence", "frames")},
            }
        )

    def fire(self, ts: float, fire: bool, reason: str, distance: float) -> None:
        if self._fh is None:
            return
        self._write({"type": "fire", "ts": round(ts, 6), "fire": bool(fire),
                     "reason": reason, "dist": round(distance, 2)})

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                self._fh.close()
                self._fh = None

    @property
    def records(self) -> int:
        return self._n


# ---------------------------------------------------------------------------
# replay
# ---------------------------------------------------------------------------
@dataclass
class RecordedSession:
    path: Path
    header: Dict[str, Any]
    frames: List[Dict[str, Any]]
    ticks: List[Dict[str, Any]]
    fires: List[Dict[str, Any]]

    @property
    def duration_s(self) -> float:
        if not self.frames:
            return 0.0
        return self.frames[-1]["ts"] - self.frames[0]["ts"]


def load_session(path: str | os.PathLike) -> RecordedSession:
    p = Path(path)
    opener = gzip.open if p.suffix == ".gz" else open
    header: Dict[str, Any] = {}
    frames: List[Dict[str, Any]] = []
    ticks: List[Dict[str, Any]] = []
    fires: List[Dict[str, Any]] = []
    with opener(p, "rt", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            kind = rec.get("type")
            if kind == "header":
                header = rec
            elif kind == "frame":
                frames.append(rec)
            elif kind == "tick":
                ticks.append(rec)
            elif kind == "fire":
                fires.append(rec)
    return RecordedSession(path=p, header=header, frames=frames, ticks=ticks, fires=fires)


def detections_of(frame: Dict[str, Any]) -> List[Detection]:
    from ..core.types import Box

    out: List[Detection] = []
    for x1, y1, x2, y2, cls_id, score in frame.get("dets", []):
        out.append(Detection(box=Box(x1, y1, x2, y2), cls_id=int(cls_id), cls_name="body", score=float(score)))
    return out


def summarize(session: RecordedSession) -> Dict[str, Any]:
    from ..core.metrics import RollingStats

    infer = RollingStats(512)
    for f in session.frames:
        infer.add(float(f.get("ms", 0.0)))
    mode_counts: Dict[str, int] = {}
    for t in session.ticks:
        mode_counts[t.get("mode", "?")] = mode_counts.get(t.get("mode", "?"), 0) + 1
    reasons: Dict[str, int] = {}
    for f in session.fires:
        reasons[f.get("reason", "?")] = reasons.get(f.get("reason", "?"), 0) + 1
    errs = RollingStats(1024)
    lats = RollingStats(1024)
    for t in session.ticks:
        if t.get("mode") in (None, "idle"):
            continue                     # no target / no output: a zero error here is an artefact
        e = t.get("err") or [0.0, 0.0]
        errs.add((e[0] ** 2 + e[1] ** 2) ** 0.5)
        lat = (t.get("lat") or {}).get("loop_ms")
        if lat:
            lats.add(float(lat))
    quality = errs.summary()
    latency = lats.summary()
    verdict: List[str] = []
    if quality["n"] == 0:
        verdict.append("nothing was ever really aimed at (no locked target, or aim was off): feel cannot be evaluated")
    elif quality["n"] > 30:
        if quality["p95"] > 60:
            verdict.append("tracking error is large: check the latency estimate, the gain and the head ratio")
        elif quality["p95"] > 25:
            verdict.append("tracking error is moderate: try raising kp or tightening the dead band")
        else:
            verdict.append("tracking quality is good")
    if latency["n"] > 10 and latency["p50"] < 20:
        verdict.append("the latency estimate is below 20 ms, physically impossible (min one frame); recheck it")
    hits = sum(1 for f in session.fires if f.get("fire"))
    if hits == 0:
        verdict.append("the session never fired: check the aim switch, the fire radius and the lock ratio")

    return {
        "path": str(session.path),
        "quality": {
            "err_p50_px": quality["p50"],
            "err_p95_px": quality["p95"],
            "err_max_px": quality["max"],
        },
        "latency_ms": {"p50": latency["p50"], "p95": latency["p95"]},
        "verdict": verdict,
        "duration_s": round(session.duration_s, 2),
        "frames": len(session.frames),
        "ticks": len(session.ticks),
        "fires": sum(1 for f in session.fires if f.get("fire")),
        "infer_ms": infer.summary(),
        "modes": mode_counts,
        "fire_reasons": reasons,
    }
