"""ONNX detector: session management, preprocessing, ROI, decoding, NMS.

Compared to v1:

* one letterbox transform used for preprocessing *and* for the inverse mapping;
* all three heads supported (e2e / yolov5 / yolov8);
* class allow/deny list is data driven, so ``teammate`` can never become a
  target and ``head`` is used when present instead of a guessed ratio;
* optional ROI crop around the crosshair: a 640x360 area upscaled into a 320x320
  network beats squeezing the whole 1280x720 frame, and it costs a quarter of
  the pixels;
* warm-up + a stable provider chain (CoreML -> CPU) with timing percentiles.
"""
from __future__ import annotations

import ast
import os
import time
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np

from ..core.config import REPO_ROOT, VisionCfg
from ..core.metrics import Metrics
from ..core.types import Box, Detection, DetectionFrame
from . import decoders
from .geometry import Transform, center_crop, make_transform

os.environ.setdefault("ORT_DISABLE_TELEMETRY", "1")

_PROVIDER_ALIASES = {
    "coreml": "CoreMLExecutionProvider",
    "cuda": "CUDAExecutionProvider",
    "tensorrt": "TensorrtExecutionProvider",
    "dml": "DmlExecutionProvider",
    "directml": "DmlExecutionProvider",
    "openvino": "OpenVINOExecutionProvider",
    "cpu": "CPUExecutionProvider",
}


def resolve_providers(requested: Iterable[str]) -> List[str]:
    import onnxruntime as ort

    available = set(ort.get_available_providers())
    chain: List[str] = []
    for name in requested:
        prov = _PROVIDER_ALIASES.get(str(name).strip().lower(), str(name))
        if prov in available and prov not in chain:
            chain.append(prov)
    if "CPUExecutionProvider" not in chain:
        chain.append("CPUExecutionProvider")
    return chain


def _parse_names(meta: Mapping[str, str]) -> Dict[int, str]:
    raw = meta.get("names")
    if not raw:
        return {}
    for parser in (ast.literal_eval,):
        try:
            value = parser(raw)
            if isinstance(value, Mapping):
                return {int(k): str(v) for k, v in value.items()}
        except (ValueError, SyntaxError):
            continue
    # "0: body, 1: head" fallback
    out: Dict[int, str] = {}
    for part in str(raw).replace("{", "").replace("}", "").split(","):
        if ":" in part:
            k, v = part.split(":", 1)
            try:
                out[int(k.strip())] = v.strip().strip("'\"")
            except ValueError:
                continue
    return out


class ClassMap:
    """Allow / deny / head classification driven by configuration."""

    def __init__(self, names: Mapping[int, str], targets: Sequence[str], ignore: Sequence[str], heads: Sequence[str] = ("head",)):
        self.names = dict(names)
        self.targets = {str(s).lower() for s in targets}
        self.ignore = {str(s).lower() for s in ignore}
        self.heads = {str(s).lower() for s in heads}
        self.id_to_name = {k: v.lower() for k, v in self.names.items()}

    def name_of(self, cls_id: int) -> str:
        return self.id_to_name.get(int(cls_id), str(cls_id))

    def is_targetable(self, cls_id: int) -> bool:
        name = self.name_of(cls_id)
        return name in self.targets and name not in self.ignore

    def is_head(self, cls_id: int) -> bool:
        return self.name_of(cls_id) in self.heads

    def target_ids(self) -> List[int]:
        return [cid for cid in self.id_to_name if self.is_targetable(cid)]

    def head_ids(self) -> List[int]:
        return [cid for cid in self.id_to_name if self.is_head(cid)]

    def summary(self) -> str:
        return "classes=" + ",".join(f"{k}:{v}" for k, v in sorted(self.id_to_name.items()))


class OnnxDetector:
    """Stateless-ish detector; ``detect()`` is called from one infer thread."""

    def __init__(self, cfg: VisionCfg, metrics: Metrics | None = None):
        import onnxruntime as ort

        self.cfg = cfg
        self.metrics = metrics
        self.model_path = self._resolve(cfg.model)
        self.names: Dict[int, str] = {}
        self.format = "e2e"
        self.sess = None
        self.input_name = ""
        self.in_w = 320
        self.in_h = 320
        self.class_map = ClassMap({}, cfg.target_classes, cfg.ignore_classes)
        self._load(ort)
        self.letterbox_mode = self._resolve_letterbox(cfg.letterbox)

    # ------------------------------------------------------------------
    @staticmethod
    def _resolve(path: str) -> Path:
        p = Path(path)
        if p.is_absolute() and p.exists():
            return p
        for base in (Path.cwd(), REPO_ROOT):
            cand = (base / path).resolve()
            if cand.exists():
                return cand
        raise FileNotFoundError(f"model not found: {path}")

    def _resolve_letterbox(self, mode: str) -> str:
        if mode in ("on", "letterbox"):
            return "letterbox"
        if mode in ("off", "stretch"):
            return "stretch"
        return "stretch" if self.format == "e2e" else "letterbox"

    def _load(self, ort) -> None:
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = int(self.cfg.intra_threads)
        opts.inter_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        opts.log_severity_level = 3

        chain = resolve_providers(self.cfg.providers)
        last_error: Exception | None = None
        for provs in (chain, ["CPUExecutionProvider"]):
            try:
                self.sess = ort.InferenceSession(str(self.model_path), sess_options=opts, providers=provs)
                self.providers = self.sess.get_providers()
                break
            except Exception as exc:  # provider missing / unsupported op
                last_error = exc
        if self.sess is None:
            raise RuntimeError(f"failed to create inference session: {last_error}")

        inp = self.sess.get_inputs()[0]
        self.input_name = inp.name
        shape = list(inp.shape)
        self.in_h = int(shape[2]) if isinstance(shape[2], int) else 640
        self.in_w = int(shape[3]) if isinstance(shape[3], int) else 640

        meta = self.sess.get_modelmeta().custom_metadata_map
        self.names = _parse_names(meta)
        out_shape = self.sess.get_outputs()[0].shape
        self.out_shape = out_shape
        self.format = decoders.infer_format(out_shape, len(self.names))
        self.class_map = ClassMap(self.names, self.cfg.target_classes, self.cfg.ignore_classes)
        self.model_name = self.model_path.name
        self._warmup()

    def _warmup(self, runs: int = 3) -> None:
        dummy = np.zeros((1, 3, self.in_h, self.in_w), dtype=np.float32)
        for _ in range(runs):
            self.sess.run(None, {self.input_name: dummy})

    # ------------------------------------------------------------------
    def describe(self) -> Dict[str, object]:
        return {
            "model": self.model_name,
            "input": f"{self.in_w}x{self.in_h}",
            "format": self.format,
            "providers": list(self.providers),
            "classes": {k: v for k, v in sorted(self.names.items())},
            "targets": self.class_map.target_ids(),
            "heads": self.class_map.head_ids(),
            "letterbox": self.letterbox_mode,
        }

    # ------------------------------------------------------------------
    def _roi(self, frame_shape: Tuple[int, int, int], hint: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float, float, float]]:
        mode = self.cfg.roi
        if mode == "off":
            return None
        h, w = frame_shape[:2]
        size = int(self.cfg.roi_size)
        if mode == "center" or hint is None:
            return center_crop(w, h, size)
        x0 = hint[0] - size * 0.5
        y0 = hint[1] - size * 0.5
        x0 = max(0.0, min(x0, w - size))
        y0 = max(0.0, min(y0, h - size))
        return x0, y0, x0 + float(size), y0 + float(size)

    def _preprocess(self, frame: np.ndarray, transform: Transform) -> np.ndarray:
        x0, y0, x1, y1 = transform.crop
        crop = frame[int(round(y0)):int(round(y1)), int(round(x0)):int(round(x1))]
        if crop.size == 0:
            crop = frame
        interp = cv2.INTER_AREA if transform.crop_w > self.in_w else cv2.INTER_LINEAR
        if self.letterbox_mode == "stretch":
            img = cv2.resize(crop, (self.in_w, self.in_h), interpolation=interp)
        else:
            nw = max(1, min(self.in_w, int(round(transform.crop_w * transform.sx))))
            nh = max(1, min(self.in_h, int(round(transform.crop_h * transform.sy))))
            resized = cv2.resize(crop, (nw, nh), interpolation=interp)
            canvas = np.full((self.in_h, self.in_w, 3), 114, dtype=np.uint8)
            ox = int(round(transform.pad_x))
            oy = int(round(transform.pad_y))
            canvas[oy:oy + nh, ox:ox + nw] = resized
            img = canvas
        blob = img[:, :, ::-1].transpose(2, 0, 1)
        return np.ascontiguousarray(blob[None], dtype=np.float32) / 255.0

    # ------------------------------------------------------------------
    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_ts: float = 0.0,
        frame_id: int = 0,
        hint: Optional[Tuple[float, float]] = None,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
    ) -> DetectionFrame:
        t0 = time.perf_counter()
        h, w = frame.shape[:2]
        roi = self._roi(frame.shape, hint)
        transform = make_transform(w, h, self.in_w, self.in_h, crop=roi, mode=self.letterbox_mode)
        blob = self._preprocess(frame, transform)
        t_pre = time.perf_counter()

        raw = self.sess.run(None, {self.input_name: blob})[0]
        t_inf = time.perf_counter()

        conf_thr = float(self.cfg.conf if conf is None else conf)
        iou_thr = float(self.cfg.iou if iou is None else iou)
        boxes, scores, cls_ids = self._decode(raw, conf_thr)
        t_dec = time.perf_counter()

        if len(boxes):
            keep = np.ones(len(boxes), dtype=bool)
            for i, cid in enumerate(cls_ids):
                if not self.class_map.is_targetable(int(cid)):
                    keep[i] = False
            boxes, scores, cls_ids = boxes[keep], scores[keep], cls_ids[keep]
        if len(boxes):
            idx = decoders.per_class_nms(boxes, scores, cls_ids, iou_thr, int(self.cfg.max_det))
            boxes, scores, cls_ids = boxes[idx], scores[idx], cls_ids[idx]
        boxes = transform.to_source_boxes(boxes) if len(boxes) else boxes

        dets: List[Detection] = []
        for (x1, y1, x2, y2), score, cid in zip(boxes, scores, cls_ids, strict=False):
            box = Box(float(x1), float(y1), float(x2), float(y2)).clip(w, h)
            if box.w < 4.0 or box.h < 8.0:
                continue
            dets.append(
                Detection(box=box, cls_id=int(cid), cls_name=self.class_map.name_of(int(cid)), score=float(score))
            )

        infer_ms = (t_inf - t_pre) * 1000.0
        total_ms = (time.perf_counter() - t0) * 1000.0
        if self.metrics is not None:
            self.metrics.observe("perception.infer_ms", infer_ms)
            self.metrics.observe("perception.total_ms", total_ms)
            self.metrics.observe("perception.pre_ms", (t_pre - t0) * 1000.0)
            self.metrics.observe("perception.post_ms", (t_dec - t_inf) * 1000.0 + (time.perf_counter() - t_dec) * 1000.0)
            self.metrics.set("perception.dets", float(len(dets)))

        return DetectionFrame(
            frame_id=frame_id,
            capture_ts=capture_ts or time.monotonic(),
            detections=tuple(dets),
            infer_ms=infer_ms,
            model=self.model_name,
            roi=tuple(roi) if roi else None,
        )

    # ------------------------------------------------------------------
    def _decode(self, raw: np.ndarray, conf: float):
        nc = max(1, len(self.names))
        if self.format == "yolov5":
            return decoders.decode_yolov5(raw, conf, nc)
        if self.format == "yolov8":
            return decoders.decode_yolov8(raw, conf, nc)
        return decoders.decode_e2e(raw, conf)

    # ------------------------------------------------------------------
    def bench(self, frames: Sequence[np.ndarray], runs: int = 5) -> Dict[str, float]:
        if not frames:
            raise ValueError("bench needs at least one frame")
        times: List[float] = []
        for _ in range(runs):
            for frame in frames:
                t0 = time.perf_counter()
                self.detect(frame)
                times.append((time.perf_counter() - t0) * 1000.0)
        times.sort()
        n = len(times)
        return {
            "n": n,
            "mean_ms": round(sum(times) / n, 3),
            "p50_ms": round(times[n // 2], 3),
            "p95_ms": round(times[min(n - 1, int(n * 0.95))], 3),
            "max_ms": round(times[-1], 3),
            "fps": round(1000.0 / (sum(times) / n), 1),
        }
