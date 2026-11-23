"""Frame sources: capture card, video file (for replay/testing) and synthetic.

A source is deliberately dumb: ``open``/``read``/``close``.  All the policy about
reconnecting, timestamps and "which frame is newest" lives in ``grabber.py`` so
that a synthetic source in CI behaves exactly like a capture card.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from ..core.config import CaptureCfg


class FrameSource:
    name = "base"

    def open(self) -> bool:  # pragma: no cover - interface
        raise NotImplementedError

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:  # pragma: no cover
        raise NotImplementedError

    def close(self) -> None:  # pragma: no cover
        pass

    def describe(self) -> dict:
        return {"source": self.name}


class CvSource(FrameSource):
    name = "capture-card"

    def __init__(self, cfg: CaptureCfg):
        self.cfg = cfg
        self.cap: Optional[cv2.VideoCapture] = None
        self._backend = 0

    def _backend_id(self) -> int:
        if os.name == "nt":
            return cv2.CAP_DSHOW
        if os.uname().sysname == "Darwin":
            return cv2.CAP_AVFOUNDATION
        return cv2.CAP_V4L2

    def open(self) -> bool:
        self.close()
        self._backend = self._backend_id()
        cap = cv2.VideoCapture(int(self.cfg.device), self._backend)
        if not cap.isOpened():
            cap.release()
            return False
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg.fourcc))
        except Exception:
            pass
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.cfg.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cfg.height))
        cap.set(cv2.CAP_PROP_FPS, int(self.cfg.fps))
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, int(self.cfg.buffer_size))
        except Exception:
            pass
        self.cap = cap
        return True

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if self.cap is None:
            return False, None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return False, None
        return True, frame

    def close(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def describe(self) -> dict:
        cap = self.cap
        info = {"source": self.name, "device": self.cfg.device}
        if cap is not None:
            info.update(
                {
                    "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    "fps": round(float(cap.get(cv2.CAP_PROP_FPS)), 2),
                }
            )
        return info


class VideoSource(FrameSource):
    """Loops a file: deterministic input for the replay harness."""

    name = "video"

    def __init__(self, path: str, loop: bool = True):
        self.path = path
        self.loop = loop
        self.cap: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.path)
        return bool(self.cap.isOpened())

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if self.cap is None:
            return False, None
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return (ok, frame) if ok else (False, None)

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


@dataclass
class SyntheticTarget:
    x: float
    y: float
    w: float = 44.0
    h: float = 96.0
    vx: float = 0.0
    vy: float = 0.0
    head: bool = True


class SyntheticSource(FrameSource):
    """Renders a dark scene with moving boxes — used by demos and smoke tests."""

    name = "synthetic"

    def __init__(self, width: int = 1280, height: int = 720, fps: int = 60, seed: int = 7):
        self.width, self.height, self.fps = width, height, fps
        self._t0 = time.monotonic()
        self._rng = np.random.default_rng(seed)
        self._targets = [
            SyntheticTarget(x=width * 0.62, y=height * 0.52, vx=-140.0, vy=0.0),
            SyntheticTarget(x=width * 0.30, y=height * 0.45, vx=110.0, vy=20.0, head=False),
        ]
        self._frame: Optional[np.ndarray] = None
        # Closed-loop sources are told where the camera points: without this the
        # scene would stay fixed while the crosshair pans, i.e. the "enemy" would
        # appear to accelerate away for ever (and the assist would chase it).
        self.cam = (0.0, 0.0)

    def set_camera(self, cam_x: float, cam_y: float) -> None:
        self.cam = (float(cam_x), float(cam_y))

    def open(self) -> bool:
        self._t0 = time.monotonic()
        return True

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        t = time.monotonic() - self._t0
        frame = np.full((self.height, self.width, 3), 18, dtype=np.uint8)
        for i, tg in enumerate(self._targets):
            x = tg.x + math.sin(t * 1.7 + i) * tg.vx * 0.6 - self.cam[0]
            y = tg.y + math.sin(t * 0.9 + i * 2.0) * tg.vy * 0.6 - self.cam[1]
            x1, y1 = int(x - tg.w / 2), int(y - tg.h / 2)
            colour = (60, 60, 200) if i == 0 else (50, 160, 90)
            cv2.rectangle(frame, (x1, y1), (x1 + int(tg.w), y1 + int(tg.h)), colour, -1)
            if tg.head:
                hr = int(tg.w * 0.55)
                cv2.circle(frame, (int(x), y1 - hr // 2), hr // 2, (70, 70, 220), -1)
        cv2.line(frame, (0, self.height // 2), (self.width, self.height // 2), (200, 200, 200), 1)
        cv2.line(frame, (self.width // 2, 0), (self.width // 2, self.height), (200, 200, 200), 1)
        self._frame = frame
        time.sleep(1.0 / max(1, self.fps))
        return True, frame

    def close(self) -> None:
        pass


def open_source(cfg: CaptureCfg, kind: str = "auto", path: str | None = None) -> FrameSource:
    if kind == "synthetic":
        return SyntheticSource(cfg.width, cfg.height, max(30, cfg.fps // 2))
    if kind == "video" and path:
        return VideoSource(path)
    return CvSource(cfg)
