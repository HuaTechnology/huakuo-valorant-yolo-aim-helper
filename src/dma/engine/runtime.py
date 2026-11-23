"""The engine: wires capture → perception → control → transport.

Thread layout (all stages are independently rate matched, nothing blocks another):

    grab thread       blocking card read, newest-frame slot, timestamps
    infer thread      YOLO on the newest frame, publishes DetectionFrame
    control thread    fixed 240 Hz tick: tracker → policy → controller → fire
    link thread       fixed 500 Hz tick: HID reports, heartbeat, device telemetry
    UI thread         polls ``snapshot()`` at 30 Hz, never touches engine state

The control tick is the only place that couples stages, and it is deterministic:
``Engine.step()`` can be driven by a test or the simulator with fake time.
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

from ..capture.grabber import FrameGrabber
from ..capture.sources import FrameSource, open_source
from ..control.controller import ControlInput, build_controller
from ..control.fire import FireController, ImpactPreview
from ..control.latency import LatencyEstimator
from ..control.policy import TargetPolicy, TargetState
from ..core.clock import DelayLine, FixedRate
from ..core.config import Settings, diff
from ..core.logging import StageLogger
from ..core.metrics import Metrics
from ..core.types import (
    ControlOutput,
    Detection,
    DetectionFrame,
    FireDecision,
    Health,
    Snapshot,
    StageStatus,
    Track,
)
from ..perception.detector import OnnxDetector
from ..perception.tracker import Tracker
from ..transport.link import LinkBase, build_link

log = StageLogger("dma.engine", min_interval_s=1.0)


class Engine:
    def __init__(
        self,
        settings: Settings,
        *,
        metrics: Metrics | None = None,
        source: FrameSource | None = None,
        link: LinkBase | None = None,
        detector: OnnxDetector | None = None,
        pace_fps: int = 0,
    ):
        self.settings = settings
        self.metrics = metrics or Metrics()
        self.detector = detector or OnnxDetector(settings.vision, self.metrics)
        self.source = source or open_source(settings.capture)
        self.grabber = FrameGrabber(self.source, settings.capture, self.metrics, pace_fps=pace_fps)
        self.link = link or build_link(settings.output, self.metrics)
        if getattr(self.link, "metrics", None) is None:
            self.link.metrics = self.metrics
        self.tracker = Tracker(settings.tracker)
        self.policy = TargetPolicy(settings.policy, head_mode=settings.vision.head_mode)
        self.controller = build_controller(settings.aim, self.metrics)
        self.fire = FireController(settings.fire, self.metrics)
        self.latency = LatencyEstimator(
            prior_s=settings.aim.latency_prior_s,
            window=settings.aim.latency_window,
            min_conf=settings.aim.latency_min_conf,
        )
        self.cam_line = DelayLine()
        self.cam = (0.0, 0.0)

        self.aim_enabled = False
        self.fire_enabled = settings.fire.mode != "off"

        self._lock = threading.Lock()
        self._snap = Snapshot()
        self._det: Optional[DetectionFrame] = None
        self._det_lock = threading.Lock()
        self._last_det_used = -1
        self._tracks: List[Track] = []
        self._target: Optional[TargetState] = None
        self._ctrl_tid: Optional[int] = None
        self._cam_meas = (0.0, 0.0)
        self._out: Optional[ControlOutput] = None
        self._fire_dec: Optional[FireDecision] = None
        self._last_tick = 0.0
        self._center = (0.0, 0.0)
        self._frame_size: Tuple[int, int] = (0, 0)

        self._running = threading.Event()
        self._infer_thread: Optional[threading.Thread] = None
        self._seq_used = -1
        self._fps = 0.0
        self._tick_rate = 0.0
        self._t_prev = 0.0
        self._metrics_t = 0.0
        self._metrics_cache: dict = {}
        self.health = Health.BOOT
        self.health_detail = ""
        self.recorder = None
        if settings.runtime.record:
            from ..telemetry.recorder import SessionRecorder

            self.recorder = SessionRecorder(settings.runtime.record_dir, settings, self.detector.describe())
        self._frame: Optional[np.ndarray] = None

    # ==================================================================
    # lifecycle
    # ==================================================================
    def start(self) -> "Engine":
        self.grabber.ensure_open()
        self.grabber.start()
        if hasattr(self.link, "start"):
            self.link.start()  # type: ignore[attr-defined]
        self._running.set()
        self._infer_thread = threading.Thread(target=self._infer_loop, name="dma-infer", daemon=True)
        self._infer_thread.start()
        self._last_tick = time.monotonic()
        self.health = Health.OK
        log.info("engine started", model=self.detector.model_name, control_hz=self.settings.runtime.control_hz)
        return self

    def run(self, duration_s: Optional[float] = None) -> None:
        """Drive the control loop in the calling thread."""
        if not self._running.is_set():
            self.start()
        t0 = time.monotonic()
        try:
            for _ in FixedRate(self.settings.runtime.control_hz):
                if not self._running.is_set():
                    break
                self.step()
                if duration_s is not None and time.monotonic() - t0 >= duration_s:
                    break
        finally:
            self.stop()

    def stop(self) -> None:
        if not self._running.is_set():
            return
        self._running.clear()
        if self._infer_thread is not None:
            self._infer_thread.join(timeout=1.0)
        self.grabber.stop()
        try:
            self.source.close()
        except Exception:
            pass
        try:
            self.link.close()
        except Exception:
            pass
        self.health = Health.STOPPED
        if self.recorder is not None:
            path = self.recorder.path
            self.recorder.close()
            log.info("session recorded", path=str(path), records=self.recorder.records)
            err = self.metrics.stats_summary("aim.err_px").get("aim.err_px", {})
            print(
                f"\nthis session: error p50 {err.get('p50', 0):.1f}px / p95 {err.get('p95', 0):.1f}px"
                f" | shots {self.fire.shots} | latency {self.latency.loop_s*1000:.1f}ms"
                f"(conf {self.latency.confidence:.2f}, rejected {self.latency.rejected})"
                f"\nsession recorded: {path}  ({self.recorder.records} records)"
            )
        log.info("engine stopped", shots=self.fire.shots)

    # ==================================================================
    # background inference
    # ==================================================================
    def _infer_loop(self) -> None:
        while self._running.is_set():
            pkt = self.grabber.latest()
            if pkt is None or pkt.seq == self._seq_used:
                time.sleep(0.001)
                continue
            self._seq_used = pkt.seq
            try:
                hint = None
                with self._det_lock:
                    if self._target is not None:
                        hint = self._target.screen_xy
                det = self.detector.detect(
                    pkt.frame,
                    capture_ts=pkt.ts,
                    frame_id=pkt.seq,
                    hint=hint,
                )
            except Exception as exc:  # never let one bad frame kill the thread
                log.throttled("infer", "error", "inference failed", err=f"{type(exc).__name__}: {exc}")
                self.metrics.inc("perception.errors")
                time.sleep(0.005)
                continue
            with self._det_lock:
                self._det = det
            self.metrics.set("perception.last_frame", det.frame_id)
            self.metrics.inc("perception.det_objects", len(det.detections))
            self.metrics.set("perception.dets", float(len(det.detections)))

    # ==================================================================
    # control tick
    # ==================================================================
    def _set_health(self, health: Health, detail: str = "") -> None:
        if health != self.health:
            log.info("health changed", health=health.value, detail=detail)
            self.metrics.inc(f"health.{health.value}")

        self.health = health
        self.health_detail = detail

    def _evaluate(self, now: float, det: Optional[DetectionFrame]) -> Tuple[Health, str]:
        if self.grabber.stale_s() * 1000.0 > self.settings.runtime.watchdog_ms:
            return Health.DEGRADED, "capture stream interrupted"
        if det is None:
            return Health.DEGRADED, "waiting for the first inference"
        if (now - det.capture_ts) > self.settings.runtime.max_infer_age_s:
            return Health.DEGRADED, "inference result stale"
        if not self.link.ok():
            return Health.DEGRADED, "output link down"
        return Health.OK, ""

    def step(self) -> Snapshot:
        now = time.monotonic()
        dt = now - self._last_tick if self._last_tick else 1.0 / self.settings.runtime.control_hz
        dt = max(1e-4, min(dt, 0.2))
        self._last_tick = now
        if self._t_prev:
            inst = 1.0 / max(1e-6, now - self._t_prev)
            self._fps = inst if self._fps == 0.0 else self._fps * 0.9 + inst * 0.1
        self._t_prev = now
        self.metrics.set("control.hz", self._fps)

        with self._det_lock:
            det = self._det
        health, detail = self._evaluate(now, det)
        self._set_health(health, detail)

        aim_cfg = self.settings.aim
        measure_offset = aim_cfg.latency_split * self.latency.loop_s

        # ---------------- perception already done; track + select ----------
        if det is not None and det.frame_id != self._last_det_used:
            self._last_det_used = det.frame_id
            pkt = self.grabber.latest()
            frame = pkt.frame if pkt is not None else None
            if frame is not None:
                h, w = frame.shape[:2]
                if (w, h) != self._frame_size:
                    self._frame_size = (w, h)
                    self._center = (w / 2.0, h / 2.0)
                    self.policy.frame_w, self.policy.frame_h = w, h
                    self.cam_line.reset()
                    self.latency.reset()
                    log.info("frame size", w=w, h=h)

            cam_meas = self.cam_line.sample(det.capture_ts - measure_offset)
            self._cam_meas = cam_meas
            world_dets = [
                Detection(
                    box=d.box.translate(cam_meas[0], cam_meas[1]),
                    cls_id=d.cls_id,
                    cls_name=d.cls_name,
                    score=d.score,
                )
                for d in det.detections
            ]
            self._tracks = self.tracker.update(world_dets, det.capture_ts)
            self.metrics.set("tracker.count", float(len(self._tracks)))

            target = self.policy.select(self._tracks, cam_meas, self._center, now)
            # Only a *measured* position is a valid delay observation: feeding
            # the estimator a dead-reckoned coasting point would teach it a
            # latency that does not exist.
            if target is not None and not target.coasting:
                self.latency.on_detection(
                    det.capture_ts,
                    target.screen_xy[0] - self._center[0],
                    target.screen_xy[1] - self._center[1],
                )
            if target is not None and target.tid != self._ctrl_tid:
                # new target: the integrator belongs to the old one, never let it
                # pull the crosshair back towards where the previous enemy was
                self.controller.reset()
                self._ctrl_tid = target.tid
            elif target is None:
                self._ctrl_tid = None
            self._target = target
            if self.recorder is not None:
                self.recorder.frame(det, cam_meas)

        # ---------------- control law --------------------------------
        out: Optional[ControlOutput] = None
        target = self._target
        can_aim = self.aim_enabled and health == Health.OK
        if target is not None:
            self.metrics.inc("policy.target_ticks")
        # The frame in hand is already old: a 60 fps capture is 16.7 ms stale on
        # average.  Total prediction horizon = loop latency + frame age, which is
        # why v1 (which used the stale position directly) always trailed a
        # strafing target by exactly one frame.
        det_age = max(0.0, now - det.capture_ts) if det is not None else 0.0
        lead_s = self.latency.loop_s + det_age

        if can_aim and target is not None:
            inp = ControlInput(
                now=now,
                dt=dt,
                world_xy=target.aim_world,
                vel_xy=(target.track.vx, target.track.vy),
                cam_now=self.cam,
                cam_meas=self._cam_meas,
                center=self._center,
                age_s=det_age + self.latency.loop_s * aim_cfg.latency_split,
                # never let a bad latency estimate collapse the effect window:
                # one 60 fps frame is the physical lower bound for "command lands"
                apply_s=max(dt, self.latency.loop_s * (1.0 - aim_cfg.latency_split), 0.020),
                sigma=target.track.sigma,
                coasting=target.coasting,
            )
            out = self.controller.step(inp)
            self.link.submit_motion(out.dx, out.dy)
            self.latency.add_motion(out.dx, out.dy)
            self.cam = (self.cam[0] + out.dx, self.cam[1] + out.dy)
            self.cam_line.push(now, self.cam[0], self.cam[1])
            if hasattr(self.source, "set_camera"):
                self.source.set_camera(*self.cam)  # type: ignore[attr-defined]
        elif target is None:
            self.controller.reset()
        self._out = out

        # ---------------- fire --------------------------------------
        decision: Optional[FireDecision] = None
        if can_aim and self.fire_enabled and target is not None and out is not None:
            lead = lead_s
            cam_impact = (self.cam[0], self.cam[1])
            target_pt = (
                target.aim_world[0] + target.track.vx * lead - cam_impact[0],
                target.aim_world[1] + target.track.vy * lead - cam_impact[1],
            )
            delta = (
                target.track.vx * lead + self._cam_meas[0] - cam_impact[0],
                target.track.vy * lead + self._cam_meas[1] - cam_impact[1],
            )
            preview = ImpactPreview(
                impact_xy=self._center,
                target_xy=target_pt,
                box=target.screen_box.translate(delta[0], delta[1]),
                residual=math.hypot(target_pt[0] - self._center[0], target_pt[1] - self._center[1]),
            )
            decision = self.fire.update(target, preview, target.track.sigma, now)
        else:
            decision = self.fire.update(None, None, 0.0, now)
        self._fire_dec = decision
        self.link.submit_button(self.fire.pressed(now) and self.fire_enabled and can_aim)
        if self.recorder is not None:
            u = (out.dx, out.dy) if out is not None else (0.0, 0.0)
            self.recorder.tick(now, self.cam, u, target.tid if target else None,
                               (out.err_x, out.err_y) if out else (0.0, 0.0),
                               out.mode.value if out else "idle", self.latency.report(aim_cfg.latency_split))
            if decision is not None and (decision.fire or decision.reason != getattr(self, "_last_fire_reason", None)):
                self.recorder.fire(now, decision.fire, decision.reason, decision.distance)
                self._last_fire_reason = decision.reason

        # ---------------- publish -----------------------------------
        self.metrics.set("aim.cam_x", self.cam[0])
        self.metrics.set("aim.cam_y", self.cam[1])
        return self._publish(now, det)

    # ==================================================================
    def _publish(self, now: float, det: Optional[DetectionFrame]) -> Snapshot:
        if now - self._metrics_t >= self.settings.runtime.metrics_interval_s:
            self._metrics_t = now
            self._metrics_cache = {
                "control": self.metrics.stats_summary("control."),
                "perception": self.metrics.stats_summary("perception."),
                "aim": self.metrics.stats_summary("aim."),
                "transport": self.metrics.stats_summary("transport."),
                "tracker": self.metrics.stats_summary("tracker."),
                "counters": self.metrics.counters(),
                "gauges": self.metrics.gauges(),
                "planner": self.link.planner.report(),
            }
        link_status = self.link.status()
        stages = [
            StageStatus("capture", self.grabber.stale_s() < 0.2, f"{self.grabber.fps:.1f}Hz"),
            StageStatus("inference", det is not None and (now - det.capture_ts) < self.settings.runtime.max_infer_age_s,
                        f"{det.infer_ms:.1f}ms" if det else "-"),
            StageStatus("link", bool(link_status.get("ok")), str(link_status.get("port", "mock"))),
        ]
        pkt = self.grabber.latest()
        snap = Snapshot(
            ts=now,
            health=self.health,
            health_detail=self.health_detail,
            aim_enabled=self.aim_enabled,
            fire_enabled=self.fire_enabled,
            fps_capture=self.grabber.fps,
            fps_control=self._fps,
            infer_ms=det.infer_ms if det else 0.0,
            detections=det.detections if det else (),
            tracks=tuple(self._tracks),
            target=self._target,
            output=self._out,
            fire=self._fire_dec,
            latency={
                **self.latency.report(self.settings.aim.latency_split),
                "stage_infer_ms": round(det.infer_ms if det else 0.0, 2),
                "frame_age_ms": round((now - det.capture_ts) * 1000.0, 1) if det else 0.0,
            },
            metrics=self._metrics_cache,
            stages=stages,
            frame=pkt.frame if pkt is not None else None,
        )
        with self._lock:
            self._snap = snap
        return snap

    # ==================================================================
    def snapshot(self) -> Snapshot:
        with self._lock:
            return self._snap

    # ==================================================================
    def set_aim_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self.aim_enabled:
            return
        self.aim_enabled = enabled
        if not enabled:
            # drop whatever has not been reported yet: re-submitting it would
            # move the crosshair *after* the operator asked us to stop
            self.link.planner.reset()
            self.controller.reset()
            self.cam = (0.0, 0.0)
            self.cam_line.reset()
        log.info("aim toggled", enabled=enabled)

    def set_fire_enabled(self, enabled: bool) -> None:
        self.fire_enabled = bool(enabled)
        log.info("fire toggled", enabled=self.fire_enabled)

    # ==================================================================
    def apply(self, new: Settings) -> List[str]:
        """Hot reload the sections that do not require reopening hardware."""
        changes = diff(self.settings, new)
        if not changes:
            return []
        notes: List[str] = []
        if any(k.startswith("vision.") for k in changes):
            notes.append("changes to the vision section need a process restart (model/thresholds load at session start)")
        if any(k.startswith("capture.") for k in changes):
            notes.append("changes to the capture section need a process restart (the capture card is open with the old settings)")
        if any(k.startswith("output.port") or k.startswith("output.baud") for k in changes):
            notes.append("serial port changes need a process restart")

        self.settings = new
        self.tracker.cfg = new.tracker
        self.policy.cfg = new.policy
        self.policy.head_mode = new.vision.head_mode
        self.policy.frame_w = self._frame_size[0] or self.policy.frame_w
        self.policy.frame_h = self._frame_size[1] or self.policy.frame_h
        if new.aim.controller != getattr(self.controller, "kind", None):
            self.controller = build_controller(new.aim, self.metrics)
        else:
            self.controller.cfg = new.aim  # type: ignore[attr-defined]
        self.fire.cfg = new.fire
        self.link.cfg = new.output
        self.link.planner.cfg = new.output
        log.info("config hot reload", changed=len(changes))
        for path, (old, now_) in changes.items():
            log.info("field changed", field=path, old=old, new=now_)
        return notes

    # ==================================================================
    def describe(self) -> dict:
        return {
            "health": self.health.value,
            "detail": self.health_detail,
            "model": self.detector.describe(),
            "capture": self.grabber.describe(),
            "link": self.link.status(),
            "latency": self.latency.report(self.settings.aim.latency_split),
            "aim_enabled": self.aim_enabled,
            "fire_enabled": self.fire_enabled,
            "shots": self.fire.shots,
            "frame_size": self._frame_size,
        }
