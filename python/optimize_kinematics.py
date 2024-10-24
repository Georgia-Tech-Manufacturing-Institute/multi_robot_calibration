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

import os
import numpy as np

from roboticstoolbox import ET, Robot
from spatialmath import SE3
from scipy.optimize import minimize

from mr_calibration.geometry import centroid, ls_registration
from mr_calibration.keyence_cmm import read_point_cloud


def modify_ets(ets, et_index, new_et):
    ets.pop(et_index)
    ets.insert(new_et, et_index)
    return ets


def set_link_lengths(ets, a):
    "a = [l1z, l2x, l3y, l3z, l4z, l5x, l5y, l6x]"
    modify_ets(ets, 1, ET.SE3(SE3(0, 0, a[0])))
    modify_ets(ets, 3, ET.SE3(SE3(a[1], 0, 0)))
    modify_ets(ets, 5, ET.SE3(SE3(0, a[2], a[3])))
    modify_ets(ets, 7, ET.SE3(SE3(0, 0, a[4])))
    modify_ets(ets, 9, ET.SE3(SE3(a[5], a[6], 0)))
    modify_ets(ets, 11, ET.SE3(SE3(a[7], 0, 0)))
    return ets


def get_link_lengths(ets):
    "l1z, l2x, l3y, l3z, l4z, l5x, l5y, l6x"
    return [
        ets[1]._T[2, 3],
        ets[3]._T[0, 3],
        ets[5]._T[1, 3],
        ets[5]._T[2, 3],
        ets[7]._T[2, 3],
        ets[9]._T[0, 3],
        ets[9]._T[1, 3],
        ets[11]._T[0, 3],
    ]


def read_tool_positions():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # tool registration data
    tool_cloud_path = os.path.join(script_path, "../CMM_data/ToolCloud/tool_cloud.csv")
    V = read_point_cloud(tool_cloud_path) / 1000

    # robot tool point clouds
    path_prefix = "~/GaTech Dropbox/Alexander Arbogast/Arbogast Research/calibration_data/rob1_kinematic_optimization_104/"

    file_names = [
        "Rob1V1.csv",
        "Rob1V2.csv",
        "Rob1V3.csv",
        "Rob1V4.csv",
        "Rob1V5.csv",
        "Rob1V6.csv",
        "Rob1V7.csv",
        "Rob1V8.csv",
        "Rob1V9.csv",
        "Rob1V10.csv",
        "Rob1V11.csv",
    ]

    cloud_paths = [os.path.join(path_prefix, fn) for fn in file_names]
    point_clouds = [read_point_cloud(cp) / 1000 for cp in cloud_paths]

    T_cmm_tool = SE3()
    for W in point_clouds:
        T_cmm_tool.append(ls_registration(V, W))
    T_cmm_tool.pop(0)

    # configs corresponding to poses
    q = np.deg2rad(
        [
            [0, 45, 30, 0, -75, 0],
            [25, 65, 30, 60, -65, 0],
            [1.141, 35.480, 19.738, -84.379, -45.001, 40.004],
            [-10.377, 121.931, -114.019, -45.710, -64.364, 20.006],
            [50, 25, 50, 75, -65, 70],
            [-25, 35, 40, -50, -112, 140],
            [40, 35, 40, 85, -30, 150],
            [165, -99, -70, 85, -118, 5],
            [-7, 15, 60, -20, -55, -60],
            [-15, 45, 20, 120, 100, 11],
            [15, 45, 20, 170, 60, 88],
        ]
    )
    return T_cmm_tool, q


def optimize_kinematics_base(nominal_robot, T_cmm_tool, q):
    """
    Optimize the parameters of a kinematic chain from measured positions. The
    cost function is the distance between the each perceived origin and the
    centroid of the perceived origins.

    Parameters
    ----------
    nominal_robot : ETS
        An elementary transform sequence with the nominal link parameters.
    T_cmm_tool : SE3
        A list of poses measured (with CMM) at the configurations in q
    q : ndarray
        An array of configurations corresponding to the poses in T_cmm_tool

    Returns
    -------
    calibrated_robot : ETS
        An elementary transform sequence with the optimal kinematic parameters
    """
    calibrated_robot = nominal_robot.copy()

    def cost(a):
        # change link lengths in ets
        set_link_lengths(calibrated_robot, a)

        T_tool_base = calibrated_robot.fkine(q).inv()
        T_cmm_base = T_cmm_tool @ T_tool_base

        center = centroid(T_cmm_base.t)
        error = np.linalg.norm(center - T_cmm_base.t, axis=1)
        c = np.linalg.norm(error)
        return c

    res = minimize(
        cost,
        x0=get_link_lengths(nominal_robot),
        method="Nelder-Mead",
        options={"fatol": 1e-12},
    )
    print(res)
    return calibrated_robot


def optimize_kinematics_tool(nominal_robot, T_cmm_tool, q):
    """
    Optimize the parameters of a kinematic chain from measured positions. The
    cost function is the distance between the each perceived origin and the
    centroid of the perceived origins.

    Parameters
    ----------
    nominal_robot : ETS
        An elementary transform sequence with the nominal link parameters.
    T_cmm_tool : SE3
        A list of poses measured (with CMM) at the configurations in q
    q : ndarray
        An array of configurations corresponding to the poses in T_cmm_tool

    Returns
    -------
    calibrated_robot : ETS
        An elementary transform sequence with the optimal kinematic parameters
    calibrated_robot = nominal_robot.copy()
    """
    calibrated_robot = nominal_robot.copy()

    def cost(a):
        # change link lengths in ets
        set_link_lengths(calibrated_robot, a)

        T_base_tool = calibrated_robot.fkine(q)

        # Transform from the first point to all other points (cmm_data)
        T_0i_g = T_cmm_tool[0] @ T_cmm_tool.inv()

        # Transform from the first point to all other points (model data)
        T_0i_e = T_base_tool[0] @ T_base_tool.inv()

        error = np.linalg.norm(T_0i_g.t - T_0i_e.t, axis=1)
        c = np.linalg.norm(error)
        return c

    res = minimize(
        cost,
        x0=get_link_lengths(nominal_robot),
        method="Nelder-Mead",
        options={"fatol": 1e-12},
    )
    print(res)
    return calibrated_robot


def calc_consecutive_errors():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # create robot
    xacro_path = script_path + "/../robot_data/za_description/urdf/za.xacro"
    robot = Robot.URDF(xacro_path)

    nominal_robot = robot.ets(end="flange")
    print(nominal_robot)

    T_cmm_tool, qq = read_tool_positions()

    kin_params = []
    for i in range(2, len(T_cmm_tool) + 1):
        print(f"\n========== Optimizing parameters with {i} positions ===========")
        T_current = T_cmm_tool[:i]
        q_current = qq[:i]
        calib_robot = optimize_kinematics_base(nominal_robot, T_current, q_current)
        calib_lens = get_link_lengths(calib_robot)
        kin_params.append(np.array(calib_lens))
        print(calib_lens)

    consec_error = [
        np.linalg.norm((i - j)[1:]) for i, j in zip(kin_params[1:], kin_params[:-1])
    ]
    print("\nChange in the parameters norm:")
    for i in range(len(consec_error)):
        print(f"{i + 2} points vs {i + 3} points: {consec_error[i]}")


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # create robot
    xacro_path = script_path + "/../robot_data/za_description/urdf/za.xacro"
    robot = Robot.URDF(xacro_path)

    nominal_robot = robot.ets(end="flange")
    print(nominal_robot)

    T_cmm_tool, qq = read_tool_positions()

    calib_robot = optimize_kinematics_base(nominal_robot, T_cmm_tool, qq)
    print(calib_robot)

    print(get_link_lengths(nominal_robot))
    print(get_link_lengths(calib_robot))


if __name__ == "__main__":
    main()
    # calc_consecutive_errors()
