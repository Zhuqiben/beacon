"""Colour threshold and depth sampling unit tests (pure functions, no ROS node).

Known HSV samples must classify correctly.
Thresholds are the shipped defaults; every function under test is a module-level
pure function in detector_node.
"""
import math

import cv2
import numpy as np
import pytest

from landmark_detector.detector_node import (DEFAULT_SEG, LANDS, SEGMENTS, add_segment,
                                             bearing_from_pixel, color_mask,
                                             percentile_depth, planar_range,
                                             split_colors)

# canonical values: [Hlo, Slo, Vlo, Hhi, Shi, Vhi]
CANON = {
    'red_a': [0, 120, 70, 10, 255, 255],
    'red_b': [170, 120, 70, 180, 255, 255],
    'green': [40, 80, 70, 85, 255, 255],
    'blue': [95, 120, 70, 130, 255, 255],
    'yellow': [20, 100, 100, 35, 255, 255],
    'white': [0, 0, 200, 180, 40, 255],
    'black': [0, 0, 0, 180, 255, 60],
}

# 640x480, 110 deg horizontal field of view
HFOV = 1.9199
K = [320.0 / math.tan(HFOV / 2.0), 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]


def build(segments=CANON):
    """Build colour -> [(low, high), ...] from the flat parameter list."""
    th = {}
    for name in SEGMENTS:
        assert add_segment(th, name, segments[name])
    return th


def hsv_of(bgr):
    return cv2.cvtColor(np.array([[bgr]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0, 0]


def classify(hsv_px, thresholds):
    """Colours matched by this pixel (normally 0 or 1 entries)."""
    px = hsv_px.astype(np.uint8).reshape(1, 1, 3)
    return [c for c, ranges in thresholds.items() if color_mask(px, ranges)[0, 0] != 0]


def _rect_contour(x0, y0, x1, y1):
    mask = np.zeros((120, 120), np.uint8)
    cv2.rectangle(mask, (x0, y0), (x1, y1), 255, -1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return cnts[0]


# ---------------- flat parameter parsing ----------------
def test_add_segment_rejects_wrong_length():
    th = {}
    assert add_segment(th, 'red_a', [0, 120, 70, 10, 255]) is False
    assert th == {}
    assert add_segment(th, 'red_a', CANON['red_a']) is True
    assert len(th['red']) == 1


def test_red_is_merged_from_two_flat_segments():
    th = build()
    assert set(th) == {'red', 'green', 'blue', 'yellow', 'white', 'black'}
    assert len(th['red']) == 2                      # red_a + red_b
    assert len(th['green']) == 1


def test_self_check_log_lists_expected_colors():
    # start-up self test: landmarks=['red','green','blue','yellow'] obstacles=['white','black']
    assert split_colors(build()) == (list(LANDS), ['white', 'black'])


def test_default_segments_also_produce_the_two_expected_lists():
    th = build({name: list(DEFAULT_SEG) for name in SEGMENTS})
    assert split_colors(th) == (list(LANDS), ['white', 'black'])


# ---------------- known HSV sample classification ----------------
@pytest.mark.parametrize('bgr, expected', [
    ((0, 0, 255), 'red'),            # pure red on the H = 0 side
    ((0, 255, 0), 'green'),
    ((255, 0, 0), 'blue'),
    ((0, 255, 255), 'yellow'),
    ((255, 255, 255), 'white'),
    ((10, 10, 10), 'black'),
])
def test_known_hsv_samples(bgr, expected):
    assert classify(hsv_of(bgr), build()) == [expected]


def test_red_b_covers_the_high_hue_half():
    # red spans the 0/180 boundary: H~178 can only be matched by red_b
    hsv = hsv_of((10, 0, 255))
    assert hsv[0] >= 170
    assert classify(hsv, build()) == ['red']

    only_a = {}
    assert add_segment(only_a, 'red_a', CANON['red_a']) is True
    assert classify(hsv, only_a) == []


@pytest.mark.parametrize('bgr', [
    (128, 128, 128),                 # grey floor V~128 must match no colour
    (150, 150, 150),
    (100, 100, 100),
])
def test_grey_floor_matches_nothing(bgr):
    assert classify(hsv_of(bgr), build()) == []


def test_hue_gap_between_green_and_blue_matches_nothing():
    # H=90 falls in the gap between green (<=85) and blue (>=95)
    assert classify(np.array([90, 255, 255], dtype=np.uint8), build()) == []


def test_low_saturation_pale_red_matches_nothing():
    assert classify(np.array([0, 60, 255], dtype=np.uint8), build()) == []


# ---------------- bearing sign ----------------
def test_bearing_is_positive_on_the_left_half_of_the_image():
    assert bearing_from_pixel(100.0, K) > 0.0       # u < cx -> left of centre -> positive
    assert bearing_from_pixel(600.0, K) < 0.0
    assert abs(bearing_from_pixel(320.0, K)) < 1e-12


def test_bearing_spans_half_fov():
    assert bearing_from_pixel(0.0, K) == pytest.approx(HFOV / 2.0)
    assert bearing_from_pixel(640.0, K) == pytest.approx(-HFOV / 2.0)


def test_bearing_magnitude_grows_with_offset():
    b = [abs(bearing_from_pixel(u, K)) for u in (0.0, 160.0, 320.0, 480.0, 640.0)]
    assert b[0] > b[1] > b[2]
    assert b[2] < b[3] < b[4]


# ---------------- depth -> planar range (the update needs the planar range) ----------------
def test_planar_range_is_identity_on_the_optical_axis():
    assert planar_range(2.0, 0.0) == pytest.approx(2.0)


def test_planar_range_recovers_the_truth_for_the_default_spawn():
    # Robot at (0,0) facing +x sees the face at x=1.8 at bearing -46 deg with an optical
    # axis depth of 1.726 m. The planar range is 1.726 / cos(46) = 2.485 m, only 0.012 m
    # off the 2.473 m face-to-camera truth, while the raw optical axis depth is 0.75 m
    # out and would blow the 0.3 m tolerance.
    for sign in (1.0, -1.0):
        est = planar_range(1.726, sign * math.radians(46.0))
        assert est == pytest.approx(2.485, abs=0.002)
        assert abs(est - 2.473) < 0.3


def test_planar_range_never_diverges_at_grazing_angles():
    for deg in (0.0, 30.0, 60.0, 85.0, 89.9, 90.0):
        assert planar_range(1.0, math.radians(deg)) <= 1.0 / 0.35 + 1e-12
    assert planar_range(1.0, math.radians(90.0)) == pytest.approx(1.0 / 0.35)


def test_planar_range_is_even_in_the_bearing_sign():
    assert planar_range(2.0, 0.7) == pytest.approx(planar_range(2.0, -0.7))
    assert planar_range(2.0, 0.0) < planar_range(2.0, 0.7)


# ---------------- depth sampling ----------------
def test_percentile_choice_biases_toward_the_near_part_of_the_roi():
    # 40% of the pixels are the face (2 m), 60% are background (9 m)
    d = np.full((10, 10), 9.0, np.float32)
    d[0:4, :] = 2.0
    assert percentile_depth(d, 0, 0, 10, 10, 0.0, 30.0, 0.4, 6.0, False) == \
        pytest.approx(2.0)
    assert percentile_depth(d, 0, 0, 10, 10, 0.0, 50.0, 0.4, 6.0, False) == \
        pytest.approx(9.0)                       # median is dragged by the background
    assert percentile_depth(d, 0, 0, 10, 10, 0.0, 90.0, 0.4, 6.0, False) == \
        pytest.approx(9.0)


def test_bbox_margin_keeps_background_out_of_the_sample():
    x, y, w, h = cv2.boundingRect(_rect_contour(36, 36, 83, 83))
    assert (x, y, w, h) == (36, 36, 48, 48)
    # the face (2 m) fills only 29.3% of the bbox, the rest is background (9 m)
    d = np.full((120, 120), 9.0, np.float32)
    d[47:73, 47:73] = 2.0
    assert percentile_depth(d, x, y, w, h, 0.25, 30.0, 0.4, 6.0, False) == \
        pytest.approx(2.0)                       # after shrinking 25% only the face remains
    assert percentile_depth(d, x, y, w, h, 0.0, 30.0, 0.4, 6.0, False) > 6.0


def test_16u_depth_is_converted_from_millimetres():
    x, y, w, h = cv2.boundingRect(_rect_contour(36, 36, 83, 83))
    d = np.full((120, 120), 9000, np.uint16)
    d[40:80, 40:80] = 2000                       # mm
    assert percentile_depth(d, x, y, w, h, 0.25, 30.0, 0.4, 6.0, True) == \
        pytest.approx(2.0)


def test_invalid_zero_depth_is_dropped():
    x, y, w, h = cv2.boundingRect(_rect_contour(36, 36, 83, 83))
    assert percentile_depth(np.zeros((120, 120), np.float32),
                            x, y, w, h, 0.25, 30.0, 0.4, 6.0, False) is None


def test_out_of_range_depth_is_dropped():
    x, y, w, h = cv2.boundingRect(_rect_contour(36, 36, 83, 83))
    d = np.full((120, 120), 20.0, np.float32)    # 20 m > max_range*2 -> invalid
    d[48:72, 48:72] = 2.0
    assert percentile_depth(d, x, y, w, h, 0.25, 30.0, 0.4, 6.0, False) == \
        pytest.approx(2.0)
    assert percentile_depth(np.full((120, 120), 20.0, np.float32),
                            x, y, w, h, 0.25, 30.0, 0.4, 6.0, False) is None
