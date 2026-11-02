"""The window must draw the box in the right place, and the path must follow the car."""
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'sim'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'target_perception_ros'))

from detection_view_node import compose  # noqa: E402
from target_perception_ros.detector import detect_hsv  # noqa: E402
from scenario import make_trajectory, path_points  # noqa: E402


def frames():
    path = os.path.join(ROOT, 'src', 'target_perception_ros', 'test', 'data', 'baseline_frames.npz')
    with np.load(path) as f:
        key = next(k for k in f.files if f[k].ndim == 4 and f[k].shape[-1] == 3)
        return f[key]


def test_box_is_drawn_around_the_detection():
    rgb = frames()[0]
    x, y, w, h = detect_hsv(rgb)
    picture = compose(rgb, (x + w / 2, y + h / 2, w, h), 0.0, 1, 0)
    assert picture.shape == rgb.shape
    assert tuple(picture[y, x + w // 2]) == (0, 220, 0)


def test_no_detection_changes_nothing_in_the_image_below_the_text():
    rgb = frames()[0]
    picture = compose(rgb, None, 0.0, 1, 1)
    assert np.array_equal(picture[100:, :, ::-1], rgb[100:])


def test_path_marker_follows_the_car():
    trajectory = make_trajectory()
    points = path_points(trajectory, 20.0, 601)
    assert points.shape == (601, 3)
    for k in (0, 150, 600):
        assert np.allclose(points[k], trajectory(k * 20.0 / 600)[0])
