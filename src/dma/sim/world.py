"""Deterministic target motion models for the closed-loop simulator.

Every scenario is a pure function of time, so a run is reproducible bit for bit
and a controller regression is a real regression, not a lucky seed.  The shapes
are chosen to stress one specific property of the control loop:

===============  =====================================================
strafe_mid       steady tracking, 0.8 Hz sine, 120 px amplitude
strafe_far       tracking at distance, prediction error dominates
jitter           noise rejection while the target is almost still
dash             large instantaneous flick, overshoot / settle time
peek             appears and disappears, coasting and re-acquire
micro_drift      very slow motion, steady state error / integrator
===============  =====================================================
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class Pose:
    cx: float          # body centre, frame coordinates
    cy: float
    w: float
    h: float
    visible: bool = True

    @property
    def head(self) -> Tuple[float, float]:
        return self.cx, self.cy - self.h * 0.38


class Scenario:
    name = "scenario"
    duration_s = 6.0

    def __init__(self, width: int = 1280, height: int = 720, seed: int = 3):
        self.width = width
        self.height = height
        self.rng = np.random.default_rng(seed)
        self.cx0 = width / 2.0
        self.cy0 = height / 2.0

    def pose(self, t: float) -> Pose:  # pragma: no cover - interface
        raise NotImplementedError

    def velocity(self, t: float, eps: float = 1e-3) -> Tuple[float, float]:
        a = self.pose(t - eps)
        b = self.pose(t + eps)
        return (b.cx - a.cx) / (2 * eps), (b.cy - a.cy) / (2 * eps)

    def __repr__(self) -> str:
        return f"<{self.name} {self.duration_s}s>"


class StrafeMid(Scenario):
    name = "strafe_mid"

    def pose(self, t: float) -> Pose:
        x = self.cx0 + 120.0 * math.sin(2 * math.pi * 0.8 * t)
        y = self.cy0 - 10.0
        return Pose(x, y, 46.0, 104.0)


class StrafeFar(Scenario):
    name = "strafe_far"
    duration_s = 7.0

    def pose(self, t: float) -> Pose:
        x = self.cx0 + 360.0 + 190.0 * math.sin(2 * math.pi * 0.55 * t + 0.6)
        y = self.cy0 + 20.0
        return Pose(x, y, 34.0, 78.0)


class Jitter(Scenario):
    name = "jitter"
    duration_s = 5.0

    def __init__(self, **kw):
        super().__init__(**kw)
        self._phase = 0.0

    def pose(self, t: float) -> Pose:
        # deterministic pseudo random walk built from incommensurate sines
        x = self.cx0 + 140.0 + 1.6 * math.sin(t * 11.3) + 1.1 * math.sin(t * 23.7 + 1.1)
        y = self.cy0 + 1.4 * math.sin(t * 9.7 + 0.4)
        return Pose(x, y, 44.0, 98.0)


class Dash(Scenario):
    """Three 60 ms flicks of ~500 px, then hold: overshoot / recovery test."""

    name = "dash"
    duration_s = 5.0

    def __init__(self, **kw):
        super().__init__(**kw)
        self._marks = (
            (0.6, self.cx0 + 260),
            (1.8, self.cx0 - 240),
            (3.0, self.cx0 + 60),
        )

    def pose(self, t: float) -> Pose:
        x = self.cx0
        for start, target in self._marks:
            if t >= start:
                a = min(1.0, (t - start) / 0.06)
                x = x + (target - x) * (a * a * (3.0 - 2.0 * a))
        return Pose(x, self.cy0, 46.0, 104.0)


class Peek(Scenario):
    name = "peek"
    duration_s = 6.0

    def pose(self, t: float) -> Pose:
        cycle = t % 1.5
        visible = cycle < 0.85
        x = self.cx0 + 300.0
        y = self.cy0
        return Pose(x, y, 44.0, 100.0, visible)


class MicroDrift(Scenario):
    name = "micro_drift"
    duration_s = 6.0

    def pose(self, t: float) -> Pose:
        x = self.cx0 + 90.0 + 6.0 * t
        y = self.cy0 + 3.0 * t
        return Pose(x, y, 44.0, 100.0)


SCENARIOS: Dict[str, Callable[..., Scenario]] = {
    "strafe_mid": StrafeMid,
    "strafe_far": StrafeFar,
    "jitter": Jitter,
    "dash": Dash,
    "peek": Peek,
    "micro_drift": MicroDrift,
}


def build(name: str, **kw) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError(f"unknown scenario {name}, available {sorted(SCENARIOS)}")
    return SCENARIOS[name](**kw)
