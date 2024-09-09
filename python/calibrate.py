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
from spatialmath import SO3, SE3
from roboticstoolbox import Robot

from mr_calibration import ls_registration, normalize


def get_data():
    # fmt:off
    # Designed points in tool frame
    V = np.array([
        [-4.079, 19.427, 10.007],  # v1
        [-4.122, 19.510, 20.063],  # v2
        [5.950, 18.968, 10.149],  # v3
        [5.856, 19.036, 20.137],  # v4

        [12.144, 12.003, 10.203],  # v5
        [12.067, 12.033, 20.247],  # v6
        [11.013, -13.005, 10.468],  # v7
        [10.916, -12.972, 20.413],  # v8

        [4.103, -19.281, 10.399],  # v9
        [4.057, -19.205, 20.430],  # v10
        [-5.956, -18.801, 10.426],  # v11
        [-6.012, -18.705, 20.391],  # v12

        [-12.311, -11.923, 10.357],  # v13
        [-12.324, -11.816, 20.325],  # v14
        [-11.146, 13.019, 10.140],  # v15
        [-11.196, 13.157, 20.113],  # v16

        [-4.643, 12.833, 26.825],  # v17
        [5.487, 12.386, 26.861],  # v18
        [-5.740, -12.115, 27.011],  # v19
        [4.243, -12.584, 27.049],  # v20
    ])
    V /= 1000

    # Robot 1 Measurement Points, output of CMM
    W1 = np.array([
        [173.535, -21.405, 430.405],  # w1
        [163.495, -20.839, 430.512],  # w2
        [173.371, -21.330, 440.624],  # w3
        [163.398, -20.785, 440.501],  # w4

        [172.980, -27.951, 447.174],  # w5
        [162.949, -27.439, 447.069],  # w6
        [171.506, -52.941, 447.352],  # w7
        [161.574, -52.433, 447.229],  # w8

        [171.270, -59.567, 440.781],  # w9
        [161.255, -59.009, 440.705],  # w10
        [171.266, -59.614, 430.710],  # w11
        [161.318, -59.040, 430.624],  # w12

        [171.669, -53.090, 424.004],  # w13
        [161.717, -52.502, 423.960],  # w14
        [173.092, -28.160, 423.863],  # w15
        [163.137, -27.544, 423.780],  # w16

        [156.418, -27.199, 430.324],  # w17
        [156.360, -27.118, 440.363],  # w18
        [155.024, -52.132, 430.534],  # w19
        [154.964, -52.075, 440.528],  # w20
    ])
    W1 /= 1000

    # Robot 2 Measurement Points, output of CMM
    W2 = np.array([
        [-220.342, -46.656, 432.081],  # w1
        [-219.297, -46.395, 431.954],  # w2
        [-220.150, -46.779, 442.095],  # w3
        [-210.169, -46.502, 441.944],  # w4

        [-220.274, -40.179, 448.688],  # w5
        [-210.266, -39.922, 448.584],  # w6
        [-220.852, -15.189, 449.015],  # w7
        [-210.917, -14.862, 448.867],  # w8

        [-221.181, -8.525, 442.480],  # w9
        [-211.157, -8.257, 442.375],  # w10
        [-221.203, -8.417, 432.413],  # w11
        [-211.244, -8.172, 432.298],  # w12

        [-221.054, -14.814, 425.636],  # w13
        [-211.133, -14.655, 425.575],  # w14
        [-220.481, -39.860, 425.350],  # w15
        [-210.509, -39.643, 425.251],  # w16

        [-203.770, -39.459, 431.782],  # w17
        [-203.677, -39.598, 441.835],  # w18
        [-204.419, -14.505, 432.144],  # w19
        [-204.323, -14.618, 442.132],  # w20
    ])
    W2 /= 1000
    # fmt: on

    # Align coordinate points frame to urdf frame
    T0 = SO3.RPY([np.pi, np.pi / 2, 0], order="xyz")
    V = (T0 * V.T).T

    return (V, W1, W2)


def main():
    # tool (V), robot1 (V1), and robot2 (V2) point clouds
    V, W1, W2 = get_data()

    # create robot
    script_path = os.path.dirname(os.path.realpath(__file__))
    xacro_path = script_path + "/../robot_data/za_description/urdf/za.xacro"
    robot = Robot.URDF(xacro_path)

    # Transformation from CMM to tool
    T_cmm_tool_rob1 = ls_registration(V, W1)
    T_cmm_tool_rob2 = ls_registration(V, W2)

    # Transformation from CMM to base
    q = np.deg2rad([0, 21, 45, 0, 360 - 66, 0])
    T_tool_base = robot.fkine(end="flange", q=q).inv()  # type: ignore
    T_cmm_base_rob1 = T_cmm_tool_rob1 * T_tool_base
    T_cmm_base_rob2 = T_cmm_tool_rob2 * T_tool_base

    # Construct world frame between robots
    t_cmm_world = 0.5 * (T_cmm_base_rob1.t + T_cmm_base_rob2.t)
    T_cmm_world = SE3.Rt(SO3().Rz(-np.pi / 2), t_cmm_world)

    # convert to world frame
    T_world_base_rob1 = T_cmm_world.inv() * T_cmm_base_rob1
    T_world_base_rob2 = T_cmm_world.inv() * T_cmm_base_rob2
    print(f"T_world_base_rob1:\n {T_world_base_rob1}")
    print(f"T_world_base_rob2:\n {T_world_base_rob2}")

    print(f"rob1 translation: {T_world_base_rob1.t}")
    print(f"rob1 orientation (rpy): {T_world_base_rob1.rpy()}")
    print(f"rob2 translation: {T_world_base_rob2.t}")
    print(f"rob2 orientation (rpy): {T_world_base_rob2.rpy()}")

    # plot
    import matplotlib.pyplot as plt

    T_world_base_rob1.plot(frame="B1")
    T_world_base_rob2.plot(frame="B2")
    plt.show()


if __name__ == "__main__":
    main()
