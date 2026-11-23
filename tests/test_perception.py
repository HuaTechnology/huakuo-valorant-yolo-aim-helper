from __future__ import annotations

import numpy as np
import pytest

from dma.core.types import Box
from dma.perception.decoders import (
    decode_e2e,
    decode_yolov5,
    decode_yolov8,
    infer_format,
    nms,
    per_class_nms,
)
from dma.perception.geometry import center_crop, make_transform


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def test_letterbox_inverse_roundtrip():
    t = make_transform(1280, 720, 320, 320, mode="letterbox")
    assert t.sx == pytest.approx(320 / 1280)
    # a box at the frame centre maps to the centre of the network input
    mx, my = t.to_model_xy(640, 360)
    assert mx == pytest.approx(160, abs=1e-6)
    assert my == pytest.approx(160, abs=1e-6)
    x, y = t.to_source_xy(mx, my)
    assert (x, y) == pytest.approx((640, 360))


def test_letterbox_boxes_roundtrip():
    t = make_transform(1280, 720, 320, 320, mode="letterbox")
    src = np.array([[100.0, 200.0, 300.0, 400.0]], dtype=np.float32)
    model = np.stack(
        [
            (src[:, 0] - t.crop_x0) * t.sx + t.pad_x,
            (src[:, 1] - t.crop_y0) * t.sy + t.pad_y,
            (src[:, 2] - t.crop_x0) * t.sx + t.pad_x,
            (src[:, 3] - t.crop_y0) * t.sy + t.pad_y,
        ],
        axis=1,
    )
    back = t.to_source_boxes(model)
    assert np.allclose(back, src, atol=1e-3)


def test_stretch_transform():
    t = make_transform(1280, 720, 640, 640, mode="stretch")
    assert t.sx == pytest.approx(0.5)
    assert t.sy == pytest.approx(640 / 720)
    assert t.pad_x == 0.0 and t.pad_y == 0.0


def test_center_crop_is_inside_frame():
    crop = center_crop(1280, 720, 512)
    assert crop == (384.0, 104.0, 896.0, 616.0)


def test_crop_transform_crop_property():
    crop = (300.0, 100.0, 900.0, 700.0)
    t = make_transform(1280, 720, 320, 320, crop=crop, mode="letterbox")
    x0, y0, x1, y1 = t.crop
    assert (x0, y0) == (300.0, 100.0)
    assert (x1 - x0, y1 - y0) == (600.0, 600.0)


def test_box_math():
    b = Box(0, 0, 10, 20)
    assert b.cx == 5 and b.cy == 10
    assert b.area == 200
    assert b.contains(5, 10)
    assert not b.contains(15, 10)
    assert b.translate(10, 0).x1 == 10
    assert b.clip(4, 4).as_tuple() == (0, 0, 4, 4)
    assert b.shrink(0.5).as_tuple() == (5.0, 10.0, 5.0, 10.0)


# ---------------------------------------------------------------------------
# decoders
# ---------------------------------------------------------------------------
def test_e2e_decode():
    raw = np.array([[[100, 100, 140, 200, 0.9, 0], [10, 10, 20, 20, 0.1, 1]]], dtype=np.float32)
    boxes, scores, cls = decode_e2e(raw, 0.5)
    assert len(boxes) == 1
    assert scores[0] == pytest.approx(0.9)
    assert cls[0] == 0


def test_e2e_cxcywh_detection():
    raw = np.array([[[120.0, 150.0, 40.0, 100.0, 0.8, 0]]], dtype=np.float32)
    boxes, _, _ = decode_e2e(raw, 0.5)
    assert boxes[0] == pytest.approx([100.0, 100.0, 140.0, 200.0])


def test_yolov5_decode():
    raw = np.zeros((1, 3, 7), dtype=np.float32)
    raw[0, 0] = [320, 320, 40, 80, 0.9, 0.8, 0.1]
    raw[0, 1] = [100, 100, 10, 10, 0.9, 0.05, 0.9]
    boxes, scores, cls = decode_yolov5(raw, 0.3, nc=2)
    assert len(boxes) == 2
    assert boxes[0] == pytest.approx([300, 280, 340, 360])
    assert cls[0] == 0 and cls[1] == 1


def test_yolov8_channel_first_and_last():
    ch_first = np.zeros((1, 9, 4), dtype=np.float32)
    ch_first[0, :4, 0] = [100, 100, 20, 40]
    ch_first[0, 4, 0] = 0.9          # body
    ch_first[0, 5, 0] = 0.2
    boxes, scores, cls = decode_yolov8(ch_first, 0.4, nc=5)
    assert len(boxes) == 1
    assert cls[0] == 0
    assert boxes[0] == pytest.approx([90, 80, 110, 120])

    ch_last = np.transpose(ch_first, (0, 2, 1))
    b2, s2, c2 = decode_yolov8(ch_last, 0.4, nc=5)
    assert len(b2) == 1 and c2[0] == 0


def test_infer_format():
    assert infer_format([1, 300, 6], 8) == "e2e"
    assert infer_format([1, 9, 2100], 5) == "yolov8"
    assert infer_format([1, 2100, 9], 5) == "yolov8"
    assert infer_format([1, 25200, 7], 2) == "yolov5"


def test_nms():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [100, 100, 110, 110]], dtype=np.float32)
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
    keep = nms(boxes, scores, 0.3)
    assert list(keep) == [0, 2]


def test_per_class_nms_keeps_different_classes():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11]], dtype=np.float32)
    scores = np.array([0.9, 0.85], dtype=np.float32)
    cls = np.array([0, 1], dtype=np.int32)
    keep = per_class_nms(boxes, scores, cls, 0.3)
    assert len(keep) == 2


def test_empty_inputs():
    empty = np.zeros((0, 4), dtype=np.float32)
    assert len(nms(empty, np.zeros(0, np.float32), 0.5)) == 0
    assert decode_e2e(np.zeros((1, 0, 6), np.float32), 0.5)[0].shape == (0, 4)
