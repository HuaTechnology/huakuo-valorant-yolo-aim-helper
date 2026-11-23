"""v1 reference controller (PID on the delayed error).

Kept **only** as the baseline for the A/B benchmark in ``dma.sim``: the whole point
of this project is that this law is measurably worse on a delayed plant, and a
claim like that needs executable evidence.  It is not used by the engine and it is
not recommended for use.

History: this is the controller the v1 single-file script shipped, tuned by hand at
60 fps with a 4 px dead zone (which existed because v1 threw away sub-pixel
remainders on the wire).
"""
import math


class AimController:
    """2-D aim controller: PD + anti-windup integral (PID).

    Why it is designed this way (against the "too slow / never sticks" and "spring oscillation" extremes):
    - v1 pure proportional gain: high gain -> overshoot oscillation under delay; low gain -> tracking lag, no lock.
    - Velocity feed-forward (ff): the real code cannot get the exact link delay, so it feeds the
      controller's own oscillation back in and diverges (verified in simulation), hence it is unused.
    - Integral term I: the classic way to kill "steady-state tracking error" without knowing the delay. It
      accumulates away the fixed lag a moving target causes -> tracking lag falls from 20-30 px to ~8px, i.e.
      it "sticks". Ships with anti-windup (integral_limit) and dead-band bleed so a lost target cannot drag.
    - Derivative term D: damping, suppresses the initial overshoot of a big jump (the spring feel).
    - Piecewise kp: a little lower far out, a little higher up close, flat overall, smooth transitions.

    Simulation result (closed loop, delay D=2~3 frames):
        D=2: initial overshoot ~13px, tracking lag ~8.7px
        D=3: initial overshoot ~44px, tracking lag ~8.4px
    """

    def __init__(
        self,
        ema_alpha_near: float = 0.55,  # close range (= head lock / fine aim) smoothing, lower is steadier
        ema_alpha_far: float = 0.92,   # far range (= drag / flick) fast response, higher is snappier
        far_alpha_th: float = 100.0,   # legacy parameter (band centre once the smooth transition is on)
        alpha_band: float = 55.0,      # width of the smooth alpha near->far band (removes hard-switch jerk)
        kp_far: float = 0.34,         # >160px (slightly faster)
        kp_mid: float = 0.34,         # 80~160
        kp_near: float = 0.34,        # 30~80
        kp_touch: float = 0.32,       # <30
        kd: float = 0.65,             # derivative damping
        ki: float = 0.15,             # integral (kills tracking lag)
        integral_limit: float = 100.0,
        integral_gate: float = 80.0,  # integrate only below this error (no flick integration, no bounce)
        deadzone: float = 4.0,
        max_step: float = 100.0,
        accel: float = 30.0,          # max per-frame velocity change (accel limit, higher is snappier)
    ):
        self.ema_alpha_near = float(ema_alpha_near)
        self.ema_alpha_far = float(ema_alpha_far)
        self.far_alpha_th = float(far_alpha_th)
        self.alpha_lo = float(far_alpha_th - alpha_band)
        self.alpha_hi = float(far_alpha_th + alpha_band)
        self.kp_far = float(kp_far)
        self.kp_mid = float(kp_mid)
        self.kp_near = float(kp_near)
        self.kp_touch = float(kp_touch)
        self.kd = float(kd)
        self.ki = float(ki)
        self.integral_limit = float(integral_limit)
        self.integral_gate = float(integral_gate)
        self.deadzone = float(deadzone)
        self.max_step = float(max_step)
        self.accel = float(accel) if accel else None
        self.reset()

    def reset(self) -> None:
        """Called on target loss/switch: clears EMA, derivative and integral so a jump cannot drag."""
        self.ema_x = None
        self.ema_y = None
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.I_x = 0.0
        self.I_y = 0.0
        self._prev_dx = 0.0
        self._prev_dy = 0.0

    def reset_integral(self) -> None:
        """On a target switch only the integral and derivative state is cleared (EMA smoothing stays
        continuous), so leftover integral cannot drag the crosshair back toward the old target."""
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.I_x = 0.0
        self.I_y = 0.0
        self._prev_dx = 0.0
        self._prev_dy = 0.0

    def update(self, aim_x: float, aim_y: float, cx: float, cy: float, dt: float = 1.0 / 60.0):
        """Take the target aim point, the screen centre and the frame interval; return clamped (dx, dy)."""
        # first frame: record state, emit nothing (stops D from differentiating the first error and ringing)
        if self.ema_x is None:
            self.ema_x = float(aim_x)
            self.ema_y = float(aim_y)
            self.prev_err_x = self.ema_x - cx
            self.prev_err_y = self.ema_y - cy
            return 0.0, 0.0

        # Adaptive smoothing: far (flick) -> high alpha, fast response; close (head lock / fine aim) -> low
        # alpha, smooth.  smoothstep blends alpha_lo~alpha_hi, so a hard switch cannot cause a velocity jerk.
        raw_dist = math.hypot(float(aim_x) - cx, float(aim_y) - cy)
        if raw_dist <= self.alpha_lo:
            alpha = self.ema_alpha_near
        elif raw_dist >= self.alpha_hi:
            alpha = self.ema_alpha_far
        else:
            t = (raw_dist - self.alpha_lo) / (self.alpha_hi - self.alpha_lo)
            t = t * t * (3.0 - 2.0 * t)   # smoothstep
            alpha = self.ema_alpha_near + (self.ema_alpha_far - self.ema_alpha_near) * t
        self.ema_x += alpha * (float(aim_x) - self.ema_x)
        self.ema_y += alpha * (float(aim_y) - self.ema_y)

        err_x = self.ema_x - cx
        err_y = self.ema_y - cy
        dist = math.hypot(err_x, err_y)

        # Dead band: hold still + slowly bleed the integral, so the loop does not keep nudging the target
        if dist <= self.deadzone:
            self.prev_err_x = err_x
            self.prev_err_y = err_y
            self.I_x *= 0.9
            self.I_y *= 0.9
            self._prev_dx = 0.0
            self._prev_dy = 0.0
            return 0.0, 0.0

        # Piecewise proportional gain
        if dist > 160.0:
            kp = self.kp_far
        elif dist > 80.0:
            kp = self.kp_mid
        elif dist > 30.0:
            kp = self.kp_near
        else:
            kp = self.kp_touch

        # Integral: accumulate only up close (tracking); no flick (large error) integration, avoids windup bounce
        if dist < self.integral_gate:
            self.I_x += err_x * dt
            self.I_y += err_y * dt
            self.I_x = max(-self.integral_limit, min(self.integral_limit, self.I_x))
            self.I_y = max(-self.integral_limit, min(self.integral_limit, self.I_y))
        else:
            self.I_x *= 0.8
            self.I_y *= 0.8

        # Derivative (per-frame difference, same units as the proportional term)
        d_err_x = err_x - self.prev_err_x
        d_err_y = err_y - self.prev_err_y

        dx = kp * err_x + self.kd * d_err_x + self.ki * self.I_x
        dy = kp * err_y + self.kd * d_err_y + self.ki * self.I_y

        # Acceleration limit: ramps the per-frame velocity up/down smoothly, no "full speed then stop" jerk
        if self.accel is not None:
            a = self.accel
            dx = max(self._prev_dx - a, min(self._prev_dx + a, dx))
            dy = max(self._prev_dy - a, min(self._prev_dy + a, dy))

        # Single-frame clamp (a jump must not fling the crosshair)
        mag = math.hypot(dx, dy)
        if mag > self.max_step:
            scale = self.max_step / mag
            dx *= scale
            dy *= scale

        self._prev_dx = dx
        self._prev_dy = dy
        self.prev_err_x = err_x
        self.prev_err_y = err_y
        return dx, dy

    def smoothed_error(self, cx: float, cy: float):
        """Return the current smoothed error distance (used by the fire gate)."""
        if self.ema_x is None:
            return float("inf")
        return math.hypot(self.ema_x - cx, self.ema_y - cy)
