"""Domain types shared by every stage of the pipeline.

Everything here is immutable (frozen dataclass / slots).  Stages exchange these
values instead of loose tuples so that a stage boundary is a *contract*: the
perception layer cannot silently hand the controller a malformed box, and the
dashboard renders exactly the same object the control law consumed.

Coordinate systems (see docs/ARCHITECTURE.md §2):

    source   pixel coordinates of the captured frame          (origin top-left)
    model    pixel coordinates of the network input (N x N)   (letterboxed)
    world    source pixels with our own camera motion removed (origin identical
             to source, but the frame of reference is the map, not the camera)
    screen   what the player sees == world - camera_displacement
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Sequence, Tuple


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class Box:
    """Axis aligned rectangle in *source* pixels."""

    x1: float
    y1: float
    x2: float
    y2: float

    # -- derived ------------------------------------------------------------
    @property
    def w(self) -> float:
        return self.x2 - self.x1

    @property
    def h(self) -> float:
        return self.y2 - self.y1

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) * 0.5

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) * 0.5

    @property
    def area(self) -> float:
        return max(0.0, self.w) * max(0.0, self.h)

    @property
    def center(self) -> Tuple[float, float]:
        return self.cx, self.cy

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return self.x1, self.y1, self.x2, self.y2

    def iou(self, other: "Box") -> float:
        ix1 = max(self.x1, other.x1)
        iy1 = max(self.y1, other.y1)
        ix2 = min(self.x2, other.x2)
        iy2 = min(self.y2, other.y2)
        iw = ix2 - ix1
        ih = iy2 - iy1
        if iw <= 0 or ih <= 0:
            return 0.0
        inter = iw * ih
        union = self.area + other.area - inter
        return inter / union if union > 0 else 0.0

    def translate(self, dx: float, dy: float) -> "Box":
        return Box(self.x1 + dx, self.y1 + dy, self.x2 + dx, self.y2 + dy)

    def scaled(self, sx: float, sy: float) -> "Box":
        return Box(self.x1 * sx, self.y1 * sy, self.x2 * sx, self.y2 * sy)

    def clip(self, w: int, h: int) -> "Box":
        return Box(
            min(max(self.x1, 0.0), float(w)),
            min(max(self.y1, 0.0), float(h)),
            min(max(self.x2, 0.0), float(w)),
            min(max(self.y2, 0.0), float(h)),
        )

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        if margin > 0.0:
            mx = self.w * margin
            my = self.h * margin
            return (self.x1 + mx) <= x <= (self.x2 - mx) and (self.y1 + my) <= y <= (self.y2 - my)
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2

    def shrink(self, frac: float) -> "Box":
        mx = self.w * frac
        my = self.h * frac
        return Box(self.x1 + mx, self.y1 + my, self.x2 - mx, self.y2 - my)


# ---------------------------------------------------------------------------
# perception
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class Detection:
    """One network detection, already mapped back to *source* pixels."""

    box: Box
    cls_id: int
    cls_name: str
    score: float

    @property
    def is_head(self) -> bool:
        return self.cls_name in ("head", "hs", "headshot")


@dataclass(frozen=True, slots=True)
class DetectionFrame:
    """Detections belonging to a single captured frame."""

    frame_id: int
    capture_ts: float          # monotonic seconds, stamped at grab
    detections: Tuple[Detection, ...] = ()
    infer_ms: float = 0.0
    model: str = ""
    roi: Tuple[float, float, float, float] | None = None

    @property
    def count(self) -> int:
        return len(self.detections)


@dataclass(frozen=True, slots=True)
class Track:
    """A persistent object hypothesis with a Kalman smoothed state.

    `box`/`velocity` are expressed in *world* coordinates: our own commanded
    camera motion has already been subtracted, therefore a stationary enemy in
    a strafing match really is stationary here.
    """

    tid: int
    cls_id: int
    cls_name: str
    box: Box
    vx: float                  # pixels / second, world frame
    vy: float
    sigma: float               # sqrt(trace(P[0:2,0:2])) position uncertainty
    hits: int
    misses: int
    age: int                   # consecutive frames this track has been alive
    score: float = 1.0
    last_ts: float = 0.0

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def is_head(self) -> bool:
        return self.cls_name in ("head", "hs", "headshot")


# ---------------------------------------------------------------------------
# control
# ---------------------------------------------------------------------------
class AimMode(str, Enum):
    OFF = "off"
    FLICK = "flick"       # large error: high gain, lead clamped
    TRACK = "track"       # inside the tracking band
    SETTLE = "settle"     # inside dead-zone: hold, integrate slowly
    COAST = "coast"       # target temporarily invisible, dead-reckon


@dataclass(frozen=True, slots=True)
class ControlOutput:
    dx: float
    dy: float
    err_x: float              # predicted error at impact time (screen px)
    err_y: float
    mode: AimMode = AimMode.OFF
    kp: float = 0.0
    lead_x: float = 0.0
    lead_y: float = 0.0
    saturated: bool = False

    @property
    def error(self) -> float:
        return math.hypot(self.err_x, self.err_y)


@dataclass(frozen=True, slots=True)
class FireDecision:
    fire: bool
    reason: str
    distance: float
    dwell_s: float


# ---------------------------------------------------------------------------
# observability
# ---------------------------------------------------------------------------
class Health(str, Enum):
    BOOT = "boot"
    OK = "ok"
    DEGRADED = "degraded"     # one stage unhealthy, aim inhibited
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass(frozen=True, slots=True)
class StageStatus:
    name: str
    ok: bool
    detail: str = ""
    latency_ms: float = 0.0


@dataclass
class Snapshot:
    """Immutable-ish view of the running engine, polled by the UI at 30 Hz."""

    ts: float = 0.0
    health: Health = Health.BOOT
    health_detail: str = ""
    aim_enabled: bool = False
    fire_enabled: bool = False
    fps_capture: float = 0.0
    fps_control: float = 0.0
    infer_ms: float = 0.0
    detections: Sequence[Detection] = field(default_factory=tuple)
    tracks: Sequence[Track] = field(default_factory=tuple)
    target: Track | None = None
    output: ControlOutput | None = None
    fire: FireDecision | None = None
    latency: Mapping[str, float] = field(default_factory=dict)
    metrics: Mapping[str, Any] = field(default_factory=dict)
    stages: Sequence[StageStatus] = field(default_factory=tuple)
    frame: Any = None          # last captured BGR frame (borrowed, read-only)

    def top_line(self) -> str:
        o = self.output
        err = o.error if o else float("nan")
        tgt = f"#{self.target.tid}" if self.target else "-"
        return (
            f"{self.health.value:<8} cap={self.fps_capture:5.1f}Hz ctl={self.fps_control:5.1f}Hz "
            f"inf={self.infer_ms:4.1f}ms tgt={tgt} err={err:5.1f}px "
            f"mode={o.mode.value if o else '-':<6}"
        )
