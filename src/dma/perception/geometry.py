"""Coordinate transforms between the captured frame and the network input.

v1 resized 1280x720 straight into a 320x320 network for the end-to-end model and
letterboxed for the YOLOv5 one, with two different inverse mappings and no shared
tests.  Detections were therefore off by a few percent of the frame — invisible
when you eyeball the preview, fatal when the controller is chasing single digit
pixel errors.

This module is the single place where scale/pad/crop live.  A ``Transform`` is
built once per frame and used both to preprocess the image and to invert every
box coming out of the network, so the two can never disagree.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from ..core.types import Box


@dataclass(frozen=True, slots=True)
class Transform:
    sx: float
    sy: float
    pad_x: float
    pad_y: float
    crop_x0: float
    crop_y0: float
    crop_w: float
    crop_h: float
    src_w: int
    src_h: int
    in_w: int
    in_h: int

    # -- model -> source ---------------------------------------------------
    def to_source_boxes(self, boxes: np.ndarray) -> np.ndarray:
        out = boxes.astype(np.float32, copy=True)
        out[:, [0, 2]] = (out[:, [0, 2]] - self.pad_x) / self.sx + self.crop_x0
        out[:, [1, 3]] = (out[:, [1, 3]] - self.pad_y) / self.sy + self.crop_y0
        return out

    def to_source_xy(self, x: float, y: float) -> Tuple[float, float]:
        return (
            (x - self.pad_x) / self.sx + self.crop_x0,
            (y - self.pad_y) / self.sy + self.crop_y0,
        )

    # -- source -> model ---------------------------------------------------
    def to_model_xy(self, x: float, y: float) -> Tuple[float, float]:
        return (
            (x - self.crop_x0) * self.sx + self.pad_x,
            (y - self.crop_y0) * self.sy + self.pad_y,
        )

    @property
    def crop(self) -> Tuple[float, float, float, float]:
        """Actual source rectangle that was fed to the network (x0, y0, x1, y1)."""
        return (self.crop_x0, self.crop_y0, self.crop_x0 + self.crop_w, self.crop_y0 + self.crop_h)

    def clip_box(self, box: Box) -> Box:
        return box.clip(self.src_w, self.src_h)


def make_transform(
    src_w: int,
    src_h: int,
    in_w: int,
    in_h: int,
    crop: Optional[Sequence[float]] = None,
    mode: str = "letterbox",
) -> Transform:
    """Build the affine map used for preprocessing + inverse mapping.

    ``crop`` is (x0, y0, x1, y1) in source pixels; ``mode`` is ``letterbox``
    (aspect preserved, grey padding) or ``stretch`` (legacy behaviour, aspect
    distorted).  ``auto`` resolves to letterbox for anchor based models and
    stretch for end-to-end models, matching how each family was trained.
    """
    if crop is None:
        cx0, cy0 = 0.0, 0.0
        cw, ch = float(src_w), float(src_h)
    else:
        cx0, cy0, cx1, cy1 = (float(v) for v in crop)
        cx0 = max(0.0, min(cx0, src_w - 2.0))
        cy0 = max(0.0, min(cy0, src_h - 2.0))
        cx1 = max(cx0 + 2.0, min(cx1, float(src_w)))
        cy1 = max(cy0 + 2.0, min(cy1, float(src_h)))
        cw, ch = cx1 - cx0, cy1 - cy0

    if mode == "stretch":
        sx, sy = in_w / cw, in_h / ch
        pad_x = pad_y = 0.0
    else:
        scale = min(in_w / cw, in_h / ch)
        sx = sy = scale
        pad_x = (in_w - cw * scale) * 0.5
        pad_y = (in_h - ch * scale) * 0.5

    return Transform(
        sx=sx,
        sy=sy,
        pad_x=pad_x,
        pad_y=pad_y,
        crop_x0=cx0,
        crop_y0=cy0,
        crop_w=cw,
        crop_h=ch,
        src_w=int(src_w),
        src_h=int(src_h),
        in_w=int(in_w),
        in_h=int(in_h),
    )


def center_crop(w: int, h: int, size: int) -> Tuple[float, float, float, float]:
    size = min(size, min(w, h))
    x0 = (w - size) * 0.5
    y0 = (h - size) * 0.5
    return x0, y0, x0 + size, y0 + size
