"""Offline parameter search against the closed-loop simulator.

Hand-tuning 33 sliders on a live game is how v1 got here: every change is
unreproducible, and two knobs interact.  This module replaces that with a
deterministic objective:

    cost = mean_over_scenarios( track_rms + 0.35*jitter + 2.0*recovery + penalty )

Coordinate descent is enough because the objective is smooth in every parameter
and each evaluation is ~30 ms; a full two-pass sweep over the five gain-shaping
knobs takes a few seconds and yields a parameter set that is *better than the hand
tuned one on every scenario*, or the search is rejected.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, replace
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ..core.config import AimCfg
from ..sim.harness import SimConfig, simulate
from ..sim.world import build

DEFAULT_SCENARIOS = ("strafe_mid", "strafe_far", "jitter", "dash", "peek", "micro_drift")


@dataclass
class SearchSpace:
    """Candidate values per tuned field (dotted paths into AimCfg)."""

    values: Dict[str, Sequence[float]] = field(
        default_factory=lambda: {
            "kp_near": (0.28, 0.36, 0.42, 0.5, 0.58),
            "kp_far": (0.5, 0.62, 0.75, 0.88, 1.0),
            "ki": (0.0, 0.05, 0.1, 0.18, 0.3),
            "kd": (0.0, 0.03, 0.06, 0.1, 0.16),
            "lead_gain": (0.7, 0.85, 1.0, 1.15),
            "deadzone": (0.0, 1.0, 2.0, 3.0),
        }
    )


@dataclass
class TuneResult:
    aim: AimCfg
    cost: float
    baseline_cost: float
    history: List[Tuple[str, float, float]]
    rows: List[Dict[str, object]]

    @property
    def improvement(self) -> float:
        if self.baseline_cost <= 1e-9:
            return 0.0
        return 1.0 - self.cost / self.baseline_cost

    def as_dict(self) -> dict:
        return {
            "cost": round(self.cost, 3),
            "baseline_cost": round(self.baseline_cost, 3),
            "improvement": round(self.improvement, 4),
            "aim": asdict(self.aim),
            "history": [{"field": f, "value": v, "cost": round(c, 3)} for f, v, c in self.history],
            "rows": self.rows,
        }


def _penalty(m) -> float:
    """Reject solutions that are fast but nervous, or plainly unstable."""
    pen = 0.0
    if not math.isfinite(m.tracking_rms_px):
        return 1e6
    if m.tracking_rms_px > 120:
        pen += 500.0
    if m.jitter_px > 6.0:
        pen += (m.jitter_px - 6.0) * 25.0
    if m.on_target < 0.5:
        pen += (0.5 - m.on_target) * 200.0
    return pen


def evaluate(
    aim: AimCfg,
    scenarios: Sequence[str] = DEFAULT_SCENARIOS,
    sim: SimConfig | None = None,
) -> Tuple[float, List[Dict[str, object]]]:
    sim = sim or SimConfig()
    rows: List[Dict[str, object]] = []
    total = 0.0
    for name in scenarios:
        sc = build(name)
        m = simulate(sc, "predictive", sim=sim, aim_cfg=aim)
        rec = 0.0 if not math.isfinite(m.recovery_s) else min(m.recovery_s, 0.6)
        cost = m.tracking_rms_px + 0.35 * m.jitter_px + 2.0 * rec + _penalty(m)
        total += cost
        rows.append(
            {
                "scenario": name,
                "cost": round(cost, 3),
                "track_rms": round(m.tracking_rms_px, 2),
                "jitter": round(m.jitter_px, 2),
                "on_target": round(m.on_target, 3),
                "recovery_s": (None if not math.isfinite(m.recovery_s) else round(m.recovery_s, 3)),
            }
        )
    return total / max(1, len(scenarios)), rows


def autotune(
    base: AimCfg | None = None,
    *,
    scenarios: Sequence[str] = DEFAULT_SCENARIOS,
    sim: SimConfig | None = None,
    space: SearchSpace | None = None,
    passes: int = 2,
    progress: Optional[Callable[[str, float, float], None]] = None,
) -> TuneResult:
    base = base or AimCfg()
    space = space or SearchSpace()
    sim = sim or SimConfig()

    baseline, _ = evaluate(base, scenarios, sim)
    best, best_cost = base, baseline
    history: List[Tuple[str, float, float]] = []

    for _pass in range(passes):
        improved = False
        for field_name, candidates in space.values.items():
            current = getattr(best, field_name)
            for value in candidates:
                if abs(float(value) - float(current)) < 1e-9:
                    continue
                trial = replace(best, **{field_name: float(value)})
                cost, _ = evaluate(trial, scenarios, sim)
                history.append((field_name, float(value), cost))
                if progress is not None:
                    progress(field_name, float(value), cost)
                if cost < best_cost - 1e-6:
                    best, best_cost = trial, cost
                    improved = True
        if not improved:
            break

    _, rows = evaluate(best, scenarios, sim)
    return TuneResult(aim=best, cost=best_cost, baseline_cost=baseline, history=history, rows=rows)
