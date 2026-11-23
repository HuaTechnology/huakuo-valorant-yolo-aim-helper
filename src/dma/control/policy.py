"""Target selection and aim-point computation.

v1 picked "nearest to crosshair, minus 30 px if it looked like a head" and had a
hard lock radius.  Two problems: it could not tell a team mate from an enemy
(the model had no such class, but the selection code had no concept of class
either), and the lock point was frozen in *screen* space while the tracker was
not, so a strafing target looked like a target that "jumped".

Here:

* candidates come from tracks, so a target keeps its identity while its box is
  momentarily missing (coasting) instead of re-scoring every frame;
* a ``head`` box that sits inside a ``body`` box is fused: one target, aim point
  at the head, no manual ``HEAD_RATIO`` calibration needed when the model has a
  head class;
* switching has hysteresis (``switch_margin``) so two overlapping enemies do not
  steal the lock from each other every frame;
* everything is expressed in world coordinates plus one camera offset, which is
  what makes the predictive controller possible.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from ..core.config import PolicyCfg
from ..core.types import Box, Track


@dataclass(frozen=True, slots=True)
class TargetState:
    track: Track
    aim_world: Tuple[float, float]
    box_world: Box
    screen_xy: Tuple[float, float]
    screen_box: Box
    is_head: bool
    score: float
    coasting: bool = False

    @property
    def tid(self) -> int:
        return self.track.tid


class TargetPolicy:
    def __init__(self, cfg: PolicyCfg, frame_w: int = 1280, frame_h: int = 720,
                 head_mode: str = "auto"):
        self.cfg = cfg
        # "ratio" is the supported mode for body-only models (7-wa): ignore the
        # head class entirely and aim at box_top + head_ratio * box_h.
        self.head_mode = head_mode
        self.frame_w = int(frame_w)
        self.frame_h = int(frame_h)
        self._current_tid: Optional[int] = None
        self._last: Optional[TargetState] = None
        self._last_seen = 0.0
        self.switches = 0

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._current_tid = None
        self._last = None
        self._last_seen = 0.0

    # ------------------------------------------------------------------
    def _aim_point(self, track: Track) -> Tuple[Tuple[float, float], bool]:
        box = track.box
        if track.is_head and self.head_mode != "ratio":
            return (box.cx, box.cy), True
        return (box.cx, box.y1 + box.h * self.cfg.head_ratio), False

    def _fuse(self, tracks: Sequence[Track]) -> List[Tuple[Track, Tuple[float, float], bool]]:
        """Collapse body/head tracks that describe the same enemy."""
        if self.head_mode == "ratio":
            bodies = [t for t in tracks if not t.is_head]
            if bodies:  # head boxes are ignored when the ratio is authoritative
                tracks = bodies
        heads = [t for t in tracks if t.is_head]
        out: List[Tuple[Track, Tuple[float, float], bool]] = []
        used_heads: set[int] = set()

        for trk in tracks:
            if trk.is_head:
                continue
            body_box = trk.box
            fused = None
            for head in heads:
                hb = head.box
                if hb.area >= body_box.area * 1.4:
                    continue
                if body_box.contains(hb.cx, hb.cy, margin=-0.02):
                    fused = head
                    break
            if fused is not None:
                used_heads.add(fused.tid)
                out.append((trk, (fused.box.cx, fused.box.cy), True, fused))
            else:
                point, is_head = self._aim_point(trk)
                out.append((trk, point, is_head, trk))

        for head in heads:
            if head.tid in used_heads:
                continue
            out.append((head, (head.box.cx, head.box.cy), True, head))
        # normalise to the 3-tuple the rest of the policy expects
        return [(t, p, h) for t, p, h, _ in out]

    # ------------------------------------------------------------------
    def select(
        self,
        tracks: Sequence[Track],
        cam: Tuple[float, float],
        center: Tuple[float, float],
        now: float,
    ) -> Optional[TargetState]:
        cfg = self.cfg
        cx, cy = center
        cam_x, cam_y = cam

        best: Optional[TargetState] = None
        best_score = math.inf
        candidates = 0

        for trk, (wx, wy), is_head in self._fuse(list(tracks)):
            if trk.hits < cfg.min_hits:
                continue
            if trk.misses > 0:
                # A coasting track is a *fallback*, never a competitor: a target
                # that teleported / reappeared elsewhere is still visible in this
                # frame, and prefering the phantom is how an assist ends up
                # aiming at empty air while the enemy walks away.
                continue
            if now - trk.last_ts > cfg.coast_s + 0.05:
                continue
            sx, sy = wx - cam_x, wy - cam_y
            sbox = trk.box.translate(-cam_x, -cam_y)
            if sbox.y1 < self.frame_h * cfg.top_mask:
                continue
            if sbox.y2 > self.frame_h * (1.0 - cfg.bottom_mask):
                continue
            if not (-0.25 * self.frame_w <= sx <= 1.25 * self.frame_w):
                continue
            dist = math.hypot(sx - cx, sy - cy)
            if dist > cfg.fov_radius:
                continue
            candidates += 1
            score = dist
            if is_head and cfg.prefer_head:
                score -= cfg.head_bonus
            if self._current_tid is not None and trk.tid != self._current_tid:
                score += cfg.switch_margin
            if score < best_score:
                best_score = score
                best = TargetState(
                    track=trk,
                    aim_world=(wx, wy),
                    box_world=trk.box,
                    screen_xy=(sx, sy),
                    screen_box=sbox,
                    is_head=is_head,
                    score=score,
                    coasting=False,
                )

        if best is not None:
            if self._current_tid != best.tid:
                self.switches += 1
            self._current_tid = best.tid
            self._last = best
            self._last_seen = now
            return best

        # nobody visible: coast on the last known state for a moment
        if self._last is not None and (now - self._last_seen) <= cfg.coast_s:
            dt = now - self._last.track.last_ts
            wx = self._last.aim_world[0] + self._last.track.vx * dt
            wy = self._last.aim_world[1] + self._last.track.vy * dt
            # Do not chase a phantom that has already been extrapolated further
            # than ``lock_radius``: that means the velocity estimate was garbage
            # (dash / teleport) and the dead-reckoned point is meaningless.
            if math.hypot(wx - self._last.aim_world[0], wy - self._last.aim_world[1]) > cfg.lock_radius:
                self._current_tid = None
                self._last = None
                return None
            coast = TargetState(
                track=self._last.track,
                aim_world=(wx, wy),
                box_world=self._last.box_world.translate(self._last.track.vx * dt, self._last.track.vy * dt),
                screen_xy=(wx - cam_x, wy - cam_y),
                screen_box=self._last.screen_box.translate(self._last.track.vx * dt, self._last.track.vy * dt),
                is_head=self._last.is_head,
                score=self._last.score,
                coasting=True,
            )
            self._last = coast
            return coast

        self._current_tid = None
        self._last = None
        return None
