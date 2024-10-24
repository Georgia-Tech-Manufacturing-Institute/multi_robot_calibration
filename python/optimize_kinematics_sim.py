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
from spatialmath import SE3, SO3
from scipy.optimize import minimize

from mr_calibration.geometry import centroid


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


def optimize_6R_base(nominal_robot, real_robot):
    calibrated_robot = nominal_robot.copy()

    def cost(a):
        set_link_lengths(calibrated_robot, a)

        T_tool_base = calibrated_robot.fkine(qq).inv()
        T_cmm_base = T_cmm_tool @ T_tool_base

        center = centroid(T_cmm_base.t)
        error = np.linalg.norm(center - T_cmm_base.t, axis=1)
        c = np.linalg.norm(error)
        return c

    qq = np.random.rand(10, 6)

    T_cmm_realbase = SE3.Rt(SO3.Rz(np.pi / 30), [0.56, 0.04, -0.32])
    T_cmm_tool = T_cmm_realbase * real_robot.fkine(qq)

    res = minimize(
        cost,
        x0=get_link_lengths(nominal_robot),
        method="Nelder-Mead",
        options={"fatol": 1e-12},
    )
    print(res)
    return calibrated_robot


def optimize_6R_tool(nominal_robot, real_robot):
    calibrated_robot = nominal_robot.copy()

    def cost(a):
        set_link_lengths(calibrated_robot, a)

        T_base_tool = calibrated_robot.fkine(qq)

        # Transform from the first point to all other points (cmm_data)
        T_0i_g = T_cmm_tool[0] @ T_cmm_tool.inv()

        # Transform from the first point to all other points (model data)
        T_0i_e = T_base_tool[0] @ T_base_tool.inv()

        error = np.linalg.norm(T_0i_g.t - T_0i_e.t, axis=1)
        c = np.linalg.norm(error)
        return c

    qq = np.random.rand(10, 6)

    T_cmm_realbase = SE3()  # SE3.Rt(SO3.Rz(np.pi / 30), [0.56, 0.04, -0.32])
    T_cmm_tool = T_cmm_realbase @ real_robot.fkine(qq)

    res = minimize(
        cost,
        x0=get_link_lengths(nominal_robot),
        method="Nelder-Mead",
        options={"fatol": 1e-12, "maxiter": 1500},
    )
    print(res)
    return calibrated_robot


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # create robot
    xacro_path = script_path + "/../robot_data/za_description/urdf/za.xacro"
    robot = Robot.URDF(xacro_path)

    nominal_robot = robot.ets(end="flange")
    print(nominal_robot)

    real_robot = nominal_robot.copy()

    # set real robot lengths
    set_link_lengths(
        real_robot, [0.455, 0.027, -0.005, 0.460, 0.034, 0.421, 0.003, 0.120]
    )

    calibrated_robot1 = optimize_6R_base(nominal_robot, real_robot)
    calibrated_robot2 = optimize_6R_tool(nominal_robot, real_robot)

    print(real_robot)
    print(calibrated_robot1)
    print(calibrated_robot2)


if __name__ == "__main__":
    main()
