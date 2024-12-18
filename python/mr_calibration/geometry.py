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
    Least-squares registration of point sets

    Find the transformation matrix that transforms the set of points `points_A`
    to `points_B` with the least-square error. In other words this provides the
    transformation (B^T_A) which expresses frame A in the coordinates of frame B.
    There should be at least 6 points in each set.

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
    transform : ndarray
        The homogeneous transformation that transforms points in points_A to
        points_B

    Raises
    ------
    ValueError
        When the determinate of the rotation matrix is farther than `thresh`
        from 1
    """
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


def centroid(point_cloud):
    """Returns the centroid of a point cloud"""
    return np.mean(point_cloud, axis=0)


def normalize(vec):
    """Returns the unit magnitude vector in the direction of vec"""
    return vec / np.linalg.norm(vec)
