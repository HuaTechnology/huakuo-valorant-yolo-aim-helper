"""A model-free detector for the synthetic scene (demo, CI, smoke tests).

It thresholds the two coloured rectangles drawn by ``SyntheticSource`` and returns
the same ``DetectionFrame`` type the ONNX path produces, so the *entire* pipeline —
tracker, policy, controller, fire gate, transport — runs for real without
onnxruntime, a capture card or a GPU.  Using it in ``aim2 run --sim`` is how a new
engineer sees the loop work in ten seconds.
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np

from ..core.types import Box, Detection, DetectionFrame


class SyntheticDetector:
    model_name = "synthetic-threshold"
    kind = "synthetic"

    def __init__(self, conf: float = 0.5):
        self.conf = conf

    # -- engine interface --------------------------------------------------
    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_ts: float = 0.0,
        frame_id: int = 0,
        hint: Optional[Tuple[float, float]] = None,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
    ) -> DetectionFrame:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (90, 80, 60), (140, 255, 255))          # blue-ish enemy
        mask |= cv2.inRange(hsv, (40, 80, 60), (85, 255, 255))          # green-ish ally
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        dets = []
        h, w = frame.shape[:2]
        for c in contours:
            x, y, bw, bh = cv2.boundingRect(c)
            if bw * bh < 400 or bh < 30:
                continue
            box = Box(float(x), float(y), float(x + bw), float(y + bh)).clip(w, h)
            dets.append(Detection(box=box, cls_id=0, cls_name="body", score=0.9))
        return DetectionFrame(
            frame_id=frame_id,
            capture_ts=capture_ts or time.monotonic(),
            detections=tuple(sorted(dets, key=lambda d: -d.box.area)),
            infer_ms=0.4,
            model=self.model_name,
        )

    def describe(self) -> dict:
        return {
            "model": self.model_name,
            "input": f"{0}x{0}",
            "format": "threshold",
            "providers": ["numpy"],
            "classes": {0: "body"},
            "targets": [0],
            "heads": [],
            "letterbox": "n/a",
        }

    def bench(self, frames, runs: int = 5) -> dict:
        times = []
        for _ in range(runs):
            for f in frames:
                t0 = time.perf_counter()
                self.detect(f)
                times.append((time.perf_counter() - t0) * 1000.0)
        times.sort()
        return {"n": len(times), "mean_ms": times[len(times) // 2], "p50_ms": times[len(times) // 2],
                "p95_ms": times[-1], "max_ms": times[-1], "fps": 1000.0 / max(1e-6, times[len(times) // 2])}
