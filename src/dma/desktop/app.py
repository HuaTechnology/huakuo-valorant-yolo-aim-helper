"""Desktop control surface (PySide6) for the v2 engine.

v1's GUI was 630 lines of hand-wired sliders for 33 magic numbers with no health
model and no way to see what the controller was doing.  This one is generated
from the configuration registry (``core.config.FIELDS``): every parameter that
exists in the schema appears with its range, unit and help text, and editing one
hot-reloads the running engine or explains why it cannot.

The window never touches engine internals — it polls ``Engine.snapshot()`` at
30 Hz, which is an immutable object.  That is why the UI cannot stall the control
loop.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ..core.config import (
    FIELDS,
    ConfigError,
    Settings,
    get_field,
    load,
    save,
    set_fields,
)
from ..core.logging import setup as setup_logging
from ..core.types import Health

SCALE = 1000  # slider int <-> float


class EngineThread(QThread):
    def __init__(self, engine):
        super().__init__()
        self.engine = engine

    def run(self) -> None:  # pragma: no cover - thread body
        self.engine.run()


class PreviewWidget(QWidget):
    """Paints the captured frame with the perception/control overlay."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self._image: Optional[QImage] = None
        self._snap = None
        # The preview is for eyeballing only: neither 30 fps nor full resolution is needed, both are pure heat.
        self._preview_hz = 30.0
        self._last_paint = 0.0
        self._img_size = (0, 0)

    def set_preview_fps(self, fps: float) -> None:
        self._preview_hz = max(0.0, float(fps))

    def set_snapshot(self, snap) -> None:
        self._snap = snap
        frame = getattr(snap, "frame", None)
        if frame is None:
            return
        if self._preview_hz <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_paint < 1.0 / self._preview_hz:
            return
        self._last_paint = now
        h, w = frame.shape[:2]
        scale = min(self.width() / max(1, w), self.height() / max(1, h)) or 1.0
        tw, th = max(1, int(w * scale)), max(1, int(h * scale))
        small = frame if (tw == w and th == h) else cv2.resize(frame, (tw, th), interpolation=cv2.INTER_LINEAR)
        small = np.ascontiguousarray(small)
        self._image = QImage(small.data, tw, th, 3 * tw, QImage.Format_BGR888).copy()
        self._img_size = (tw, th)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(16, 16, 18))
        if self._image is None:
            p.setPen(QColor(140, 140, 150))
            p.drawText(self.rect(), Qt.AlignCenter, "No frames: check the capture card / the model")
            return
        scaled = self._image.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        x0 = (self.width() - scaled.width()) // 2
        y0 = (self.height() - scaled.height()) // 2
        p.drawImage(x0, y0, scaled)
        snap = self._snap
        if snap is None or snap.frame is not None:
            pass
        if snap is not None:
            self._draw_overlay(p, snap, x0, y0, scaled.width(), scaled.height())

    def _draw_overlay(self, p: QPainter, snap, x0: int, y0: int, w: int, h: int) -> None:
        fh, fw = (snap.frame.shape[:2] if snap.frame is not None else (720, 1280))
        sx, sy = w / fw, h / fh

        def m(x, y):
            return x0 + x * sx, y0 + y * sy

        # detections
        p.setPen(QPen(QColor(70, 200, 120), 1))
        for d in snap.detections:
            x1, y1 = m(d.box.x1, d.box.y1)
            x2, y2 = m(d.box.x2, d.box.y2)
            p.drawRect(int(x1), int(y1), int(x2 - x1), int(y2 - y1))
        # tracks
        p.setPen(QPen(QColor(120, 170, 255), 1, Qt.DashLine))
        for t in snap.tracks:
            x1, y1 = m(t.box.x1, t.box.y1)
            x2, y2 = m(t.box.x2, t.box.y2)
            p.drawRect(int(x1), int(y1), int(x2 - x1), int(y2 - y1))
        tgt = snap.target
        if tgt is not None:
            p.setPen(QPen(QColor(255, 90, 90), 2))
            x1, y1 = m(tgt.screen_box.x1, tgt.screen_box.y1)
            x2, y2 = m(tgt.screen_box.x2, tgt.screen_box.y2)
            p.drawRect(int(x1), int(y1), int(x2 - x1), int(y2 - y1))
            ax, ay = m(*tgt.screen_xy)
            p.drawLine(int(ax) - 6, int(ay), int(ax) + 6, int(ay))
            p.drawLine(int(ax), int(ay) - 6, int(ax), int(ay) + 6)
        # crosshair + deadzone
        cx, cy = m(fw / 2, fh / 2)
        p.setPen(QPen(QColor(255, 255, 255), 1))
        p.drawLine(int(cx) - 10, int(cy), int(cx) + 10, int(cy))
        p.drawLine(int(cx), int(cy) - 10, int(cx), int(cy) + 10)
        p.setPen(QPen(QColor(255, 220, 90, 120), 1, Qt.DotLine))
        p.drawEllipse(int(cx) - 24, int(cy) - 24, 48, 48)


class ParamRow(QWidget):
    changed = Signal(str, float)

    def __init__(self, path: str, value: float, parent=None):
        super().__init__(parent)
        meta = FIELDS[path]
        self.path = path
        self._scale = 1.0 / max(1e-9, meta.step)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(meta.lo * self._scale))
        self.slider.setMaximum(int(meta.hi * self._scale))
        self.slider.setValue(int(value * self._scale))
        self.slider.setToolTip(meta.help or meta.label)
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(3 if meta.step < 0.1 else 2)
        self.spin.setRange(meta.lo, meta.hi)
        self.spin.setSingleStep(meta.step)
        self.spin.setValue(value)
        self.spin.setSuffix(f" {meta.unit}" if meta.unit else "")
        self.spin.setMinimumWidth(96)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)

    def _on_slider(self, raw: int) -> None:
        value = raw / self._scale
        if abs(value - self.spin.value()) > 1e-9:
            self.spin.blockSignals(True)
            self.spin.setValue(value)
            self.spin.blockSignals(False)
        self.changed.emit(self.path, value)

    def _on_spin(self, value: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(int(value * self._scale))
        self.slider.blockSignals(False)
        self.changed.emit(self.path, value)

    def set_value(self, value: float) -> None:
        self.spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.spin.setValue(value)
        self.slider.setValue(int(value * self._scale))
        self.spin.blockSignals(False)
        self.slider.blockSignals(False)


class MainWindow(QMainWindow):
    def __init__(self, engine, settings: Settings, settings_path: Optional[Path] = None):
        super().__init__()
        self.engine = engine
        self.settings = settings
        self.settings_path = settings_path
        self.setWindowTitle("Hongwa - Industrial Aim Console v2")
        self.resize(1440, 900)
        self._rows: Dict[str, ParamRow] = {}

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # -- top bar -----------------------------------------------------
        bar = QHBoxLayout()
        self.health = QLabel("BOOT")
        self.health.setFont(QFont("Menlo", 13, QFont.Bold))
        self.aim_btn = QPushButton("Aim OFF")
        self.aim_btn.setCheckable(True)
        self.fire_btn = QPushButton("Fire OFF")
        self.fire_btn.setCheckable(True)
        self.model_combo = QComboBox()
        for p in sorted(Path("models").glob("*.onnx")):
            self.model_combo.addItem(p.name, str(p))
        current = Path(settings.vision.model).name
        if self.model_combo.findText(current) >= 0:
            self.model_combo.setCurrentText(current)
        self.save_btn = QPushButton("Save settings")
        self.reload_btn = QPushButton("Reload")
        for w in (QLabel("Health"), self.health, self.aim_btn, self.fire_btn,
                  QLabel("Model"), self.model_combo, self.save_btn, self.reload_btn):
            bar.addWidget(w)
        bar.addStretch(1)
        root.addLayout(bar)

        # -- preview + params -------------------------------------------
        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.preview = PreviewWidget()
        self.preview.set_preview_fps(settings.runtime.preview_fps)
        left_layout.addWidget(self.preview, 1)
        self.hud = QLabel("Waiting for data...")
        self.hud.setFont(QFont("Menlo", 12))
        left_layout.addWidget(self.hud)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFont(QFont("Menlo", 10))
        left_layout.addWidget(self.log, 0)
        split.addWidget(left)

        right = QScrollArea()
        right.setWidgetResizable(True)
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("Search parameters...")
        panel_layout.addWidget(self.filter)
        groups: Dict[str, QGroupBox] = {}
        for path, meta in FIELDS.items():
            box = groups.get(meta.group)
            if box is None:
                box = QGroupBox(meta.group)
                form = QFormLayout(box)
                form.setLabelAlignment(Qt.AlignRight)
                box._form = form  # type: ignore[attr-defined]
                groups[meta.group] = box
                panel_layout.addWidget(box)
            row = ParamRow(path, get_field(settings, path))
            row.changed.connect(self._on_param)
            self._rows[path] = row
            groups[meta.group]._form.addRow(meta.label, row)  # type: ignore[attr-defined]
        panel_layout.addStretch(1)
        right.setWidget(panel)
        split.addWidget(right)
        split.setSizes([900, 540])
        root.addWidget(split, 1)

        self.statusBar().setSizeGripEnabled(False)
        self._thread: Optional[EngineThread] = None
        self._start_engine()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)
        self.filter.textChanged.connect(self._filter_rows)

    # ------------------------------------------------------------------
    def _start_engine(self) -> None:
        self._thread = EngineThread(self.engine)
        self._thread.start()

    def _on_param(self, path: str, value: float) -> None:
        try:
            new = set_fields(self.settings, {path: value})
        except ConfigError as exc:
            self._append(f"[config rejected] {path}={value}: {exc}")
            self._rows[path].set_value(get_field(self.settings, path))
            return
        self.settings = new
        notes = self.engine.apply(new)
        for note in notes:
            self._append(f"[restart required] {note}")

    def _filter_rows(self, text: str) -> None:
        text = text.strip().lower()
        for path, row in self._rows.items():
            meta = FIELDS[path]
            visible = (not text) or text in path.lower() or text in meta.label.lower()
            row.setVisible(visible)

    def _append(self, text: str) -> None:
        self.log.appendPlainText(f"{time.strftime('%H:%M:%S')} {text}")

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        snap = self.engine.snapshot()
        self.preview.set_snapshot(snap)
        color = {
            Health.OK: "#3ddc84",
            Health.DEGRADED: "#ffb020",
            Health.FAILED: "#ff5252",
            Health.BOOT: "#888",
            Health.STOPPED: "#666",
        }.get(snap.health, "#888")
        self.health.setText(snap.health.value.upper())
        self.health.setStyleSheet(f"color:{color}")
        # One-way sync: the UI only mirrors engine state and must never emit toggled,
        # otherwise every refresh (33 ms) re-presses a switch the user just turned on.
        for btn, on, label in (
            (self.aim_btn, snap.aim_enabled, "Aim"),
            (self.fire_btn, snap.fire_enabled, "Fire"),
        ):
            btn.blockSignals(True)
            btn.setChecked(on)
            btn.setText(f"{label} {'ON' if on else 'OFF'}")
            btn.blockSignals(False)
        lat = snap.latency or {}
        out = snap.output
        hud = (
            f"cap {snap.fps_capture:5.1f}Hz  ctl {snap.fps_control:5.1f}Hz  inf {snap.infer_ms:4.1f}ms   "
            f"err {out.error if out else float('nan'):5.1f}px  mode {out.mode.value if out else '-':<6}  "
            f"lat {lat.get('loop_ms', 0):5.1f}ms(conf {lat.get('confidence', 0):.2f})   "
            f"det {len(snap.detections)}  trk {len(snap.tracks)}  "
            f"fire {snap.fire.reason if snap.fire else '-'}"
        )
        if snap.health_detail:
            hud += f"   [{snap.health_detail}]"
        self.hud.setText(hud)

    # ------------------------------------------------------------------
    def _apply_native_toggles(self) -> None:
        self.engine.set_aim_enabled(self.aim_btn.isChecked())
        self.engine.set_fire_enabled(self.fire_btn.isChecked())

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            self.engine.stop()
            if self._thread is not None:
                self._thread.wait(1000)
        finally:
            super().closeEvent(event)


def _wire(window: MainWindow) -> None:
    window.aim_btn.toggled.connect(lambda on: window.engine.set_aim_enabled(on))
    window.fire_btn.toggled.connect(lambda on: window.engine.set_fire_enabled(on))
    window.save_btn.clicked.connect(lambda: _save(window))
    window.reload_btn.clicked.connect(lambda: _reload(window))


def _save(window: MainWindow) -> None:
    path = save(window.settings, window.settings_path)
    window._append(f"settings saved: {path}")


def _reload(window: MainWindow) -> None:
    window.settings = load(window.settings_path)
    for path, row in window._rows.items():
        row.set_value(get_field(window.settings, path))
    window._append("settings reloaded")


def main(argv: Optional[list[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(prog="aim2-gui")
    parser.add_argument("--settings", default=None)
    parser.add_argument("--sim", action="store_true", help="synthetic frames, no hardware needed")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--aim", action="store_true", help="enable aim at startup (same as v1's one-key run)")
    args = parser.parse_args(argv)

    setup_logging(level="INFO", log_dir=None, console=False)
    settings = load(args.settings)
    if args.dry_run:
        settings = set_fields(settings, {"output.dry_run": True})

    from ..capture.sources import SyntheticSource
    from ..engine.runtime import Engine
    from ..sim.fake_detector import SyntheticDetector
    from ..transport.link import build_link

    source = None
    detector = None
    if args.sim:
        source = SyntheticSource(settings.capture.width, settings.capture.height, 60)
        detector = SyntheticDetector()
    link = build_link(settings.output, force_mock=args.sim or settings.output.dry_run)
    engine = Engine(settings, source=source, link=link, detector=detector)

    app = QApplication.instance() or QApplication(sys.argv[:1])
    window = MainWindow(engine, settings, Path(args.settings) if args.settings else None)
    _wire(window)
    if args.aim:
        window.aim_btn.setChecked(True)      # the toggled signal is wired to engine.set_aim_enabled
    window.show()
    return app.exec()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
