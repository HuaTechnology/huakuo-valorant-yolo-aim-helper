"""Output head decoders + NMS for the three model families in ``models/``.

    e2e      (1, 300, 6)        [x1, y1, x2, y2, conf, cls]  already NMS-ed
    yolov5   (1, 25200, 5+nc)   anchor based, objectness * class score
    yolov8   (1, 4+nc, 2100)    anchor free, no objectness, needs transpose

v1 only knew the first two and simply could not load ``sjz320v8_kenny.onnx`` —
the only model that has a real ``head`` class and a ``teammate`` class.  That
single missing decoder is why the handover doc had to invent ``HEAD_RATIO`` and
why the aim drifted onto team mates.  Everything here is pure numpy so it can be
unit tested without onnxruntime.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np
import numpy.typing as npt

Boxes = npt.NDArray[np.float32]


def infer_format(out_shape, n_classes: int) -> str:
    """Decide the decoder from the output tensor shape and the class table."""
    if out_shape is None:
        return "e2e"
    dims = [d for d in out_shape if isinstance(d, int)]
    if len(dims) == 3:
        _, a, b = dims
        if b == 6 or a == 6:
            return "e2e"
        if n_classes and a == 4 + n_classes and b != 4 + n_classes:
            return "yolov8"
        if n_classes and b == 4 + n_classes and a != 4 + n_classes:
            return "yolov8"
        if n_classes and b == 5 + n_classes:
            return "yolov5"
        if n_classes and a == 5 + n_classes:
            return "yolov5"
        # fall back to the classic "big dim is anchors" heuristic
        if a < b:
            return "yolov8"
        return "yolov5"
    return "e2e"


def decode_e2e(raw: npt.NDArray, conf: float) -> Tuple[Boxes, npt.NDArray, npt.NDArray]:
    """End-to-end head: boxes are already in model-input pixels."""
    pred = raw.reshape(-1, raw.shape[-1])
    if pred.shape[0] == 0:
        return np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, np.int32)
    scores = pred[:, 4].astype(np.float32)
    keep = scores >= conf
    pred = pred[keep]
    if pred.shape[0] == 0:
        return np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, np.int32)
    boxes = pred[:, :4].astype(np.float32)
    # some exports emit cx,cy,w,h instead of x1,y1,x2,y2
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]
    if np.mean(w <= 1.0) > 0.5 or np.mean(h <= 1.0) > 0.5:
        cx, cy, bw, bh = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
    return boxes, pred[:, 4].astype(np.float32), pred[:, 5].astype(np.int32)


def decode_yolov5(raw: npt.NDArray, conf: float, nc: int) -> Tuple[Boxes, npt.NDArray, npt.NDArray]:
    """YOLOv5 head: ``[cx, cy, w, h, obj, cls...]`` in model-input pixels."""
    pred = raw.reshape(-1, raw.shape[-1])
    if pred.shape[1] < 6:
        return np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, np.int32)
    cls_scores = pred[:, 5:]
    if cls_scores.shape[1] > nc:  # export padded with extra classes
        cls_scores = cls_scores[:, :nc]
    obj = pred[:, 4:5]
    scores_all = obj * cls_scores
    cls_ids = scores_all.argmax(axis=1).astype(np.int32)
    scores = scores_all[np.arange(len(scores_all)), cls_ids].astype(np.float32)
    keep = scores >= conf
    if not np.any(keep):
        return np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, np.int32)
    p = pred[keep]
    cx, cy, bw, bh = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1).astype(np.float32)
    return boxes, scores[keep], cls_ids[keep]


def decode_yolov8(raw: npt.NDArray, conf: float, nc: int) -> Tuple[Boxes, npt.NDArray, npt.NDArray]:
    """YOLOv8 head: ``[cx, cy, w, h, cls...]`` with no objectness channel.

    Accepts both ``(1, 4+nc, anchors)`` and ``(1, anchors, 4+nc)`` layouts.
    """
    arr = raw
    if arr.ndim == 3:
        arr = arr[0]
    if arr.ndim != 2:
        raise ValueError(f"yolov8 head: unexpected shape {raw.shape}")
    ch = 4 + nc
    if arr.shape[0] == ch and arr.shape[1] != ch:
        arr = arr.T
    elif arr.shape[1] == ch:
        pass
    elif arr.shape[0] < arr.shape[1]:
        arr = arr.T  # best effort
    pred = arr
    cls_scores = pred[:, 4:4 + nc]
    cls_ids = cls_scores.argmax(axis=1).astype(np.int32)
    scores = cls_scores[np.arange(len(cls_scores)), cls_ids].astype(np.float32)
    keep = scores >= conf
    if not np.any(keep):
        return np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, np.int32)
    p = pred[keep]
    cx, cy, bw, bh = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1).astype(np.float32)
    return boxes, scores[keep], cls_ids[keep]


def nms(
    boxes: npt.NDArray,
    scores: npt.NDArray,
    iou_thr: float,
    max_det: int = 300,
) -> npt.NDArray:
    """Greedy NMS over xyxy boxes; returns kept indices (descending score)."""
    if len(boxes) == 0:
        return np.zeros(0, dtype=np.int64)
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size and len(keep) < max_det:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)
        order = rest[iou <= iou_thr]
    return np.asarray(keep, dtype=np.int64)


def per_class_nms(
    boxes: npt.NDArray,
    scores: npt.NDArray,
    cls_ids: npt.NDArray,
    iou_thr: float,
    max_det: int = 300,
) -> npt.NDArray:
    if len(boxes) == 0:
        return np.zeros(0, dtype=np.int64)
    keep_all = []
    for c in np.unique(cls_ids):
        idx = np.flatnonzero(cls_ids == c)
        kept = nms(boxes[idx], scores[idx], iou_thr, max_det)
        keep_all.append(idx[kept])
    if not keep_all:
        return np.zeros(0, dtype=np.int64)
    keep = np.concatenate(keep_all)
    return keep[np.argsort(scores[keep])[::-1]][:max_det]
