"""Camera-frame to world-coordinate transform unit tests (pure functions, no ROS graph).

Also covers the bearing sign convention and the map->odom composition.
"""
import math

import numpy as np
import pytest

from ekf_slam.ekf_core import (EKFSLAM, map_to_odom, quat_to_yaw, world_from_camera,
                               wrap, yaw_to_quat)


class Quat:
    """Minimal stand-in for geometry_msgs/Quaternion, for quat_to_yaw."""

    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


def observation_of(x, y, th, lx, ly):
    """World coordinates -> body-frame observation (range, bearing): the inverse of
    world_from_camera."""
    dx, dy = lx - x, ly - y
    return math.hypot(dx, dy), float(wrap(math.atan2(dy, dx) - th))


# ---------------- camera frame -> world frame ----------------
def test_world_from_camera_translates_and_rotates():
    # Robot at (1, 1, pi/2) looking 2 m ahead -> world (1, 3).
    lx, ly = world_from_camera(1.0, 1.0, math.pi / 2.0, 2.0, 0.0)
    assert lx == pytest.approx(1.0, abs=1e-9)
    assert ly == pytest.approx(3.0, abs=1e-9)


def test_left_side_bearing_goes_to_the_left_in_the_map():
    # Facing +x, "left" is +y, so a positive bearing must go left; a sign error mirrors
    # the whole map.
    lx, ly = world_from_camera(0.0, 0.0, 0.0, 2.0, 0.1)
    assert lx == pytest.approx(2.0 * math.cos(0.1))
    assert ly > 0.0

    lx2, ly2 = world_from_camera(0.0, 0.0, 0.0, 2.0, -0.1)
    assert lx2 == pytest.approx(lx)
    assert ly2 == pytest.approx(-ly)


def test_world_from_camera_round_trip_through_observation():
    for (x, y, th) in ((0.0, 0.0, 0.0), (1.2, -0.7, 0.4), (-2.0, 3.0, -1.1),
                       (0.5, 0.5, 3.0)):                       # 3.0 needs wrap
        for (r, phi) in ((1.0, 0.0), (2.5, 0.3), (4.0, -0.8), (0.5, 1.2)):
            lx, ly = world_from_camera(x, y, th, r, phi)
            r2, phi2 = observation_of(x, y, th, lx, ly)
            assert r2 == pytest.approx(r)
            assert phi2 == pytest.approx(phi, abs=1e-9)


def test_world_from_camera_matches_ekf_landmark_initialisation():
    # init_landmark and the transform must give the same mean, but only after the range
    # is converted to the same convention (the detector reports camera -> near face while
    # the map stores origin -> centre).
    e = EKFSLAM()
    e.x[0], e.x[1], e.x[2] = 1.2, -0.7, 0.4
    e.init_landmark('red', 2.5, 0.3)
    ang = float(wrap(0.4 + 0.3))
    lx, ly = world_from_camera(1.2, -0.7, 0.4, 2.5 + e.geometry_bias(ang, 2.5), 0.3)
    gx, gy = e.landmark('red')
    assert gx == pytest.approx(lx)
    assert gy == pytest.approx(ly)
    # Without the conversion the landmark lands more than 0.25 m towards the robot.
    ox, oy = world_from_camera(1.2, -0.7, 0.4, 2.5, 0.3)
    assert math.hypot(gx - ox, gy - oy) > 0.25


def test_heading_wrap_keeps_transform_continuous_across_pi():
    # th = pi+0.01 and th = -pi+0.01 are the same heading (2pi apart) and must give the
    # same result.
    ax, ay = world_from_camera(0.0, 0.0, math.pi + 0.01, 2.0, 0.0)
    bx, by = world_from_camera(0.0, 0.0, -math.pi + 0.01, 2.0, 0.0)
    assert ax == pytest.approx(bx, abs=1e-9)
    assert ay == pytest.approx(by, abs=1e-9)
    assert float(wrap(math.pi + 0.01)) == pytest.approx(-math.pi + 0.01, abs=1e-12)
    # Headings 0.01 rad apart give y values 0.02 m apart: continuous, no jump.
    lx, ly = world_from_camera(0.0, 0.0, math.pi - 0.01, 2.0, 0.0)
    assert lx == pytest.approx(bx, abs=1e-3)
    assert ly == pytest.approx(0.02, abs=1e-3)


# ---------------- quaternion ----------------
def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -0.7, 0.0, 0.7, 3.0):
        qx, qy, qz, qw = yaw_to_quat(yaw)
        assert qx == 0.0 and qy == 0.0
        assert math.hypot(qz, qw) == pytest.approx(1.0)
        assert quat_to_yaw(Quat(qx, qy, qz, qw)) == pytest.approx(yaw)
        assert quat_to_yaw((qx, qy, qz, qw)) == pytest.approx(yaw)


# ---------------- map->odom ----------------
def test_map_to_odom_is_identity_when_the_estimator_agrees_with_odometry():
    x, y, th = 1.0, 0.5, 0.7
    mo_x, mo_y, mo_th = map_to_odom(x, y, th, x, y, th)
    assert mo_x == pytest.approx(0.0, abs=1e-9)
    assert mo_y == pytest.approx(0.0, abs=1e-9)
    assert mo_th == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('x, y, th, ob_x, ob_y, ob_th', [
    (1.0, 2.0, 0.3, 0.5, -0.2, 0.1),
    (-1.5, 0.4, -0.6, 0.2, 0.9, -2.8),
    (0.0, 0.0, 0.0, 1.0, 1.0, 0.5),
])
def test_map_to_odom_composes_back_to_the_estimated_pose(x, y, th, ob_x, ob_y, ob_th):
    # map->odom composed with odom->base must recover the EKF's map->base pose.
    mo_x, mo_y, mo_th = map_to_odom(x, y, th, ob_x, ob_y, ob_th)
    rx = mo_x + math.cos(mo_th) * ob_x - math.sin(mo_th) * ob_y
    ry = mo_y + math.sin(mo_th) * ob_x + math.cos(mo_th) * ob_y
    assert rx == pytest.approx(x, abs=1e-9)
    assert ry == pytest.approx(y, abs=1e-9)
    assert float(wrap(mo_th + ob_th)) == pytest.approx(float(wrap(th)), abs=1e-9)


def test_map_to_odom_is_not_a_plain_translation_difference():
    # The rotation term cannot be dropped: with ob_th != 0 the plain (x-ob_x, y-ob_y) is
    # wrong.
    mo_x, mo_y, _ = map_to_odom(1.0, 2.0, 0.3, 0.5, -0.2, 0.9)
    assert abs(mo_x - (1.0 - 0.5)) > 0.1
    assert abs(mo_y - (2.0 + 0.2)) > 0.1


def test_map_to_odom_heading_is_wrapped():
    _, _, mo_th = map_to_odom(0.0, 0.0, 3.0, 0.0, 0.0, -3.0)   # 3.0+3.0 > pi
    assert -math.pi <= mo_th < math.pi
    assert mo_th == pytest.approx(float(wrap(6.0)))


def test_landmark_geometry_is_consistent_with_a_rotated_robot():
    # Facing 90 degrees, a landmark on the left must land along +x.
    e = EKFSLAM()
    e.x[2] = math.pi / 2.0
    e.init_landmark('green', 3.0, 0.2)
    gx, gy = e.landmark('green')
    assert gx < 0.0                                    # facing +y, left is -x
    r, phi = observation_of(0.0, 0.0, math.pi / 2.0, gx, gy)
    # The observation comes back as an origin -> centre range, larger than the detector's
    # camera -> near face range by the bias.
    r_eff = 3.0 + e.geometry_bias(float(wrap(math.pi / 2.0 + 0.2)), 3.0)
    assert r == pytest.approx(r_eff)
    assert phi == pytest.approx(0.2)
    assert np.hypot(gx, gy) == pytest.approx(r_eff)
