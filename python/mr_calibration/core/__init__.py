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

from .dh_robot import *
from .geometry import *
from .keyence_cmm import *

import numpy as np
from spatialmath import SE3
from scipy.optimize import minimize


def calibrate(tool_cloud, robot_clouds, robot_configs, robot):
    if len(robot_clouds) != len(robot_configs):
        raise ValueError("Mismatch in length of point cloud and configuration arrays")

    # transformations from CMM to base
    T_cmm_base = []
    for i in range(len(robot_clouds)):
        try:
            T_cmm_tool_r = ls_registration(tool_cloud, robot_clouds[i])
        except ValueError:
            print(f"Invalid point cloud registration for robot {i}")
            return None

        T_tool_base_r = robot.fk(robot_configs[i]).inv()
        T_cmm_base_r = T_cmm_tool_r * T_tool_base_r
        T_cmm_base.append(T_cmm_base_r)

    return T_cmm_base


def tool_calibration(robot: Robot, configs: np.ndarray, initial: np.ndarray):
    def cost(x):
        flange_poses = robot.fk(configs)
        tool_positions = flange_poses * x
        t = np.sum(np.std(tool_positions, axis=1))
        return t

    res = minimize(
        cost,
        x0=initial,
        method="BFGS",
        options={"fatol": 1e-12},
    )
    print(res)
    return res.x


def create_new_world_frame(T_B1_B2: SE3):
    """Creates a frame with:
    - an origin centered at the average translation
    - a y-axis (y) pointing from B2 to B1
    - a z-axis (z) that is the average of Z1, Z2 projected on
      the plane with normal y

    Parameters
    ----------
        T_B1_B2 : SE3
            The transformation(s) from the first base frame to
            all other baseframes

    Returns
    -------
        (T_W_B1, T_W_B2) : tuple
            The baseframe transformations in the new world frame
    """

    vec = normalize(T_B1_B2.t)
    t_new_world = 0.5 * T_B1_B2.t

    unit_y = -vec
    unit_z = normalize(0.5 * (np.array([0, 0, 1]) + T_B1_B2.A[:3, 2]))
    unit_z = normalize(unit_z - np.dot(unit_z, vec) * vec)
    unit_x = normalize(np.cross(unit_y, unit_z))

    T_B1_W = SE3.Rt(np.column_stack((unit_x, unit_y, unit_z)), t_new_world)
    T_W_B1 = T_B1_W.inv()

    return T_W_B1, T_W_B1 * T_B1_B2
