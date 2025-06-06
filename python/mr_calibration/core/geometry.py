# Copyright 2024 Andrew Schneider, Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from spatialmath import SO3, SE3


def ls_registration(points_A, points_B, thresh=1e-2):
    """
    Passive least-squares registration of point sets

    This finds the SE(3) matrix (R, t) that transforms points_A to
    points_B with the least-square error.

    points_B = R * points_A + t

    For the same point cloud expressed in two frames (A, B), this returns the
    transformation of frame A with respect to frame B (B^T_A). There should be
    at least 6 points in each set.

    Parameters
    ----------
    points_A : array_like
        The first list of points (Nx3)
    points_B : array_like
        The second list of points (Nx3)
    thresh : float
        The maximum allowed deviation from 1 for the determinate of the rotation
        matrix.

    Returns
    -------
    transform : SE3
        The homogeneous transformation that transforms points in points_A to
        points_B

    Raises
    ------
    ValueError
        When the determinate of the rotation matrix is farther than `thresh`
        from 1
    """
    assert points_A.shape == points_B.shape

    centroid_A = np.mean(points_A, axis=0)
    centroid_B = np.mean(points_B, axis=0)

    H = (points_A - centroid_A).T @ (points_B - centroid_B)
    U, _, V = np.linalg.svd(H)

    R = V.T @ U.T
    if np.abs(1 - np.linalg.det(R)) > thresh:
        raise ValueError("Determinate threshold exceeded")

    transform = SE3.Rt(SO3(R), centroid_B - R @ centroid_A)
    return transform


def line_intersection(p0, v0, p1, v1):
    """
    Find the point of intersection between two lines.

    Parameters
    ----------
    p0 : ndarray
        A point on the first line
    v0 : ndarray
        The direction of the first line
    p1 : ndarracenter : ndarray
        The center of the circle passing through a, b, c
    radius: float
        The radius of the circle passing through a, b, cy
        A point on the second line
    v1 : ndarray
        The direction of the second line

    Returns
    -------
    intersection : ndarray

    """
    params = np.linalg.pinv(np.array([v0, -v1]).T) @ (p1 - p0)
    return p0 + params[0] * v0


def three_point_circle(a, b, c):
    """
    Get the parameterized circle that passes through the points a, b, and c

    Parameters
    ----------
    a : ndarray
        The first point
    b : ndarray
        The second point
    c : ndarray
        The third point

    Returns
    -------
    center : ndarray
        The center of the circle passing through a, b, c
    radius: float
        The radius of the circle passing through a, b, c
    """
    v0, v1 = b - a, c - b

    # plane normal
    n = np.cross(v0, v1)

    # midpoints
    p0 = 0.5 * (a + b)
    p1 = 0.5 * (b + c)

    # axes directions
    dp0 = np.cross(v0, n)
    dp1 = np.cross(v1, n)

    # center
    center = line_intersection(p0, dp0, p1, dp1)
    return center, np.linalg.norm(a - center)


def create_cuboid(l, w, h):
    """
    Create the 8 corner points of a 3D cuboid centered at the origin.

    Parameters
    ----------
    l : float
        Length of the cuboid along the x-axis.
    w : float
        Width of the cuboid along the y-axis.
    h : float
        Height of the cuboid along the z-axis.

    Returns
    -------
    points : ndarray
        An (8, 3) array containing the (x, y, z) coordinates of the cube's
        corners. The cube is centered at the origin (0, 0, 0).
    """
    l2, w2, h2 = l / 2, w / 2, h / 2
    points = np.array(
        [
            [l2, w2, h2],
            [l2, -w2, h2],
            [-l2, -w2, h2],
            [-l2, w2, h2],
            [-l2, w2, -h2],
            [l2, w2, -h2],
            [l2, -w2, -h2],
            [-l2, -w2, -h2],
        ]
    )
    return points


def average_every_m_points(mat, M):
    """
    Averages every M consecutive rows in a 2D array of 3D points.

    Parameters
    ----------
    mat : ndarray
        A 2D NumPy array of shape (N, 3), where each row represents a 3D point.
    M : int
        The number of consecutive points to average. Must evenly divide N.

    Returns
    -------
    ndarray
        A 2D NumPy array of shape (N // M, 3), where each row is the mean of M
        consecutive rows from the input.

    Raises
    ------
    ValueError
        If the number of rows N is not divisible by M.
    """
    N = mat.shape[0]
    if N % M != 0:
        raise ValueError(
            f"The number of rows N must be divisible by M, but N = {N} and M = {M}"
        )

    reshaped_arr = mat.reshape(N // M, M, 3)
    averaged_arr = reshaped_arr.mean(axis=1)
    return averaged_arr


def centroid(point_cloud):
    """Returns the centroid of a point cloud"""
    return np.mean(point_cloud, axis=0)


def normalize(vec):
    """Returns the unit magnitude vector in the direction of vec"""
    return vec / np.linalg.norm(vec)
