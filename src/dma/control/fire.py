"""Trigger discipline.

v1 fired whenever ``hypot(aim - centre) < FIRE_RADIUS`` and a 216 ms cooldown had
elapsed.  That fires mid-swing, fires while the tracker is guessing (coasting),
and fires at a box the controller is about to leave.  It also ignores the single
most important fact about a delayed loop: the shot lands where the crosshair
*will be* once the commands already in flight have been applied.

v3 fires only when all of these hold:

1. the crosshair *at impact time* (after every queued displacement) is inside
   the target box, shrunk by ``inset`` — the tighter head box is used when the
   model knows heads, which is what makes a head-only policy safe;
2. the remaining residual (what the controller could not remove) is inside the
   fallback radius, so a shot is never taken mid-swing;
3. the target was continuously eligible for ``dwell_s``;
4. the Kalman position uncertainty is below ``max_sigma`` (no guessing);
5. the target is not coasting;
6. the cooldown elapsed and, in ``single`` mode, this acquisition has not fired.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..core.config import FireCfg
from ..core.metrics import Metrics
from ..core.types import Box, FireDecision
from .policy import TargetState


@dataclass(frozen=True, slots=True)
class ImpactPreview:
    """Where the crosshair will be, versus where the enemy will be."""

    impact_xy: tuple[float, float]
    target_xy: tuple[float, float]
    box: Box
    residual: float


class FireController:
    def __init__(self, cfg: FireCfg, metrics: Metrics | None = None):
        self.cfg = cfg
        self.metrics = metrics
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._press_until = 0.0
        self._last_fire = -1e9
        self._dwell_start: Optional[float] = None
        self._fired_tid: Optional[int] = None
        self._last_tid: Optional[int] = None
        self.shots = 0
        self.last_reason = "init"

    # ------------------------------------------------------------------
    def pressed(self, now: float) -> bool:
        return now < self._press_until

    # ------------------------------------------------------------------
    def update(
        self,
        target: Optional[TargetState],
        preview: Optional[ImpactPreview],
        sigma: float,
        now: float,
    ) -> FireDecision:
        cfg = self.cfg
        if cfg.mode == "off":
            return self._decide(False, "off", float("inf"), 0.0)
        if target is None or preview is None:
            self._dwell_start = None
            return self._decide(False, "no-target", float("inf"), 0.0)

        if target.tid != self._last_tid:
            self._dwell_start = None
            self._fired_tid = None
        self._last_tid = target.tid

        if target.coasting:
            self._dwell_start = None
            return self._decide(False, "coasting", preview.residual, 0.0)

        if sigma > cfg.max_sigma:
            self._dwell_start = None
            return self._decide(False, "uncertain", preview.residual, 0.0)

        # Two independent ways to be "on target":
        #   * the impact point is inside the box, shrunk by ``inset``; or
        #   * the impact point is within ``radius`` of the aim point.
        # Requiring both sounds stricter but is wrong near the top edge: with
        # ``head_ratio`` aiming at the neck, a 4 px error is inside the box yet
        # outside a 10%-shrunk box, so the trigger would never release.
        inner = preview.box.shrink(min(0.45, max(0.0, cfg.inset)))
        inside = inner.contains(*preview.impact_xy)
        near = preview.residual <= cfg.radius
        if not (inside and near):
            self._dwell_start = None
            return self._decide(False, "off-target", preview.residual, 0.0)

        if self._dwell_start is None:
            self._dwell_start = now
        dwell = now - self._dwell_start
        if dwell < cfg.dwell_s:
            return self._decide(False, "settling", preview.residual, dwell)

        if now - self._last_fire < cfg.cooldown_s:
            return self._decide(False, "cooldown", preview.residual, dwell)

        if cfg.mode == "single" and self._fired_tid == target.tid:
            return self._decide(False, "already-fired", preview.residual, dwell)

        self._press_until = now + cfg.tap_s
        self._last_fire = now
        self._fired_tid = target.tid if cfg.mode == "single" else None
        self.shots += 1
        if self.metrics is not None:
            self.metrics.inc("fire.shots")
        return self._decide(True, "fire", preview.residual, dwell)

    # ------------------------------------------------------------------
    def _decide(self, fire: bool, reason: str, distance: float, dwell: float) -> FireDecision:
        self.last_reason = reason
        if self.metrics is not None:
            self.metrics.inc(f"fire.reason.{reason}")
        return FireDecision(fire=fire, reason=reason, distance=distance, dwell_s=dwell)
