"""The aim controller: delay compensated, gain scheduled, saturating safely.

The plant
---------
Screen error is an integrator driven by our own output, delayed by the loop
latency ``L``::

    e(t) = p(t - L_measure) - c(t - L_apply)      # what the sensor saw / where we are

For a *delayed integrator* the optimal command is deadbeat: compute where the
enemy will be when the command actually lands and ask for exactly that
displacement::

    u = k · (p̂(t + L_apply) - c(t) - centre)

with ``k = 1`` being the theoretical optimum and therefore the stability limit.
We use ``k`` in ``[0.50, 0.85]``, scheduled by error magnitude:

* far away (a flick) the enemy position is dominated by its own velocity, which
  the Kalman filter estimates well, so we can be aggressive;
* close in, the residual is dominated by detector noise and by our own lag
  estimation error, so the gain drops and the integrator takes over.  This is the
  regime where v1 either oscillated ("spring") or never converged.

Everything is clamped in the *screen* domain, so a saturated flick cannot
overshoot the target: the step limit is applied after the gain, never before.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from ..core.clock import clamp, smoothstep
from ..core.config import AimCfg
from ..core.metrics import Metrics
from ..core.types import AimMode, ControlOutput


@dataclass(frozen=True, slots=True)
class ControlInput:
    now: float
    dt: float
    world_xy: Tuple[float, float]        # aim point, world frame
    vel_xy: Tuple[float, float]          # world frame px/s
    cam_now: Tuple[float, float]         # accumulated commanded displacement
    cam_meas: Tuple[float, float]        # camera displacement at sensor time
    center: Tuple[float, float]
    age_s: float = 0.0                   # age of the sensor sample at ``now``
    apply_s: float = 0.0165              # command -> visible effect (rate norm)
    sigma: float = 0.0
    coasting: bool = False


class PredictiveAim:
    """Predictive PD+I with conditional integration and slew limiting."""

    kind = "predictive"

    def __init__(self, cfg: AimCfg, metrics: Metrics | None = None):
        self.cfg = cfg
        self.metrics = metrics
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._i = [0.0, 0.0]
        self._prev_err = [0.0, 0.0]
        self._prev_out = [0.0, 0.0]
        self._ema = [0.0, 0.0]
        self._inside_deadzone = False
        self._settled = 0
        self._time = 0.0

    # ------------------------------------------------------------------
    @property
    def settled(self) -> bool:
        return self._settled >= 3

    def step(self, inp: ControlInput) -> ControlOutput:
        cfg = self.cfg
        dt = max(1e-4, min(float(inp.dt), 0.2))
        # Rate normalisation.  The control law is written as "total displacement
        # per command-to-effect window", not "per tick": the same gains must mean
        # the same thing whether the control thread runs at 60 Hz or 240 Hz.
        # Without this factor a 240 Hz loop moves the camera 4x too fast and the
        # delay turns a stable loop into a limit cycle (this is exactly why the
        # v1 gains only worked at 60 fps).
        scale = clamp(dt / max(1e-3, float(inp.apply_s)), 0.02, 1.0)

        # Feed-forward = the displacement the enemy itself will travel during the
        # window in which our command takes effect.  It is added *outside* the
        # proportional gain: scaling it by kp is what makes a proportional loop
        # lag a constant-velocity target by v*apply/kp (v1's entire "tracking lag"
        # complaint).  Clamped in pixels so one bad velocity estimate cannot
        # become a 600 px overshoot.
        age = max(0.0, float(inp.age_s))
        ff_x = clamp(inp.vel_xy[0] * inp.apply_s * cfg.lead_gain, -cfg.lead_max, cfg.lead_max)
        ff_y = clamp(inp.vel_xy[1] * inp.apply_s * cfg.lead_gain, -cfg.lead_max, cfg.lead_max)
        if inp.coasting:
            ff_x *= 0.4
            ff_y *= 0.4
        # error at *now*: the sample is ``age`` old, so extrapolate it forward
        tx = inp.world_xy[0] + inp.vel_xy[0] * age
        ty = inp.world_xy[1] + inp.vel_xy[1] * age
        ex = tx - inp.cam_now[0] - inp.center[0]
        ey = ty - inp.cam_now[1] - inp.center[1]
        dist = math.hypot(ex, ey)

        # -- dead band with hysteresis ----------------------------------
        if self._inside_deadzone:
            if dist > cfg.deadzone_exit:
                self._inside_deadzone = False
        elif dist <= cfg.deadzone:
            self._inside_deadzone = True

        if self._inside_deadzone:
            self._settled += 1
            self._i[0] *= 0.92
            self._i[1] *= 0.92
            self._prev_err = [ex, ey]
            self._prev_out = [0.0, 0.0]
            if self.metrics is not None:
                self.metrics.inc("aim.hold_ticks")
            return ControlOutput(0.0, 0.0, ex, ey, AimMode.SETTLE, 0.0, 0.0, 0.0, False)

        self._settled = 0
        kp = cfg.kp_near + (cfg.kp_far - cfg.kp_near) * smoothstep(cfg.near_px, cfg.far_px, dist)
        if inp.coasting:
            kp *= 0.7
        mode = AimMode.COAST if inp.coasting else (AimMode.FLICK if dist >= cfg.far_px else AimMode.TRACK)

        # -- integral only where it helps, with conditional integration --
        if dist < cfg.integral_gate:
            self._i[0] = clamp(self._i[0] + ex * dt, -cfg.integral_limit, cfg.integral_limit)
            self._i[1] = clamp(self._i[1] + ey * dt, -cfg.integral_limit, cfg.integral_limit)
        else:
            self._i[0] *= 0.85
            self._i[1] *= 0.85

        d_x = cfg.kd * (ex - self._prev_err[0])
        d_y = cfg.kd * (ey - self._prev_err[1])

        ux = scale * (kp * ex + cfg.ki * self._i[0] + ff_x) + d_x
        uy = scale * (kp * ey + cfg.ki * self._i[1] + ff_y) + d_y

        # -- slew limit (jerk) before the magnitude clamp ----------------
        if cfg.max_accel > 0.0:
            a = cfg.max_accel * scale
            ux = clamp(ux, self._prev_out[0] - a, self._prev_out[0] + a)
            uy = clamp(uy, self._prev_out[1] - a, self._prev_out[1] + a)

        max_u = cfg.max_step * scale
        mag = math.hypot(ux, uy)
        saturated = mag > max_u
        if saturated:
            k = max_u / mag
            ux *= k
            uy *= k
            # Anti wind-up by headroom projection: the integrator may only keep
            # as much charge as the part of the output budget the proportional
            # and derivative terms did not already consume.  Without this the
            # integrator keeps winding during a saturated flick and the target
            # gets a "bounce" the moment the flick ends.
            if cfg.ki > 1e-6:
                headroom = max(0.0, max_u - math.hypot(scale * (kp * ex + ff_x) + d_x, scale * (kp * ey + ff_y) + d_y))
                lim = headroom / cfg.ki
                self._i[0] = clamp(self._i[0], -lim, lim)
                self._i[1] = clamp(self._i[1], -lim, lim)
            if self.metrics is not None:
                self.metrics.inc("aim.saturated")

        # -- output smoothing (group delay (1-α)/α · dt, i.e. sub-millisecond)
        alpha = clamp(cfg.output_ema, 0.2, 1.0)
        self._ema[0] = self._ema[0] + (ux - self._ema[0]) * alpha
        self._ema[1] = self._ema[1] + (uy - self._ema[1]) * alpha
        out_x, out_y = self._ema

        self._prev_err = [ex, ey]
        self._prev_out = [out_x, out_y]
        self._time = inp.now
        if self.metrics:
            self.metrics.observe("aim.err_px", dist)
            self.metrics.observe("aim.kp", kp)
        return ControlOutput(out_x, out_y, ex, ey, mode, kp, ff_x, ff_y, saturated)


class LegacyAim:
    """Adapter around the v1 ``AimController`` (kept for A/B and rollback)."""

    kind = "legacy"

    def __init__(self, cfg: AimCfg, aim_cfg_kwargs: Optional[dict] = None):
        from .legacy_pid import AimController

        self.cfg = cfg
        self._ctrl = AimController(**(aim_cfg_kwargs or {}))
        self._last_err = (0.0, 0.0)

    def reset(self) -> None:
        self._ctrl.reset()

    def step(self, inp: ControlInput) -> ControlOutput:
        sx = inp.world_xy[0] - inp.cam_meas[0]
        sy = inp.world_xy[1] - inp.cam_meas[1]
        dx, dy = self._ctrl.update(sx, sy, inp.center[0], inp.center[1], inp.dt)
        err = (sx - inp.center[0], sy - inp.center[1])
        self._last_err = err
        return ControlOutput(dx, dy, err[0], err[1], AimMode.TRACK, 0.0, 0.0, 0.0, False)


def build_controller(cfg: AimCfg, metrics: Metrics | None = None):
    if cfg.controller == "legacy":
        return LegacyAim(cfg)
    return PredictiveAim(cfg, metrics)
