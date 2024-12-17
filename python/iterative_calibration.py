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
import argparse
import numpy as np
import pandas as pd
from spatialmath import SO3, SE3

import matplotlib.pyplot as plt
from mr_calibration import ls_registration
from mr_calibration.core import create_new_world_frame


def parse_data(data_path):
    """Returns averaged point cloud"""
    data = pd.read_csv(data_path)
    points_avg = data.groupby("Element Name")[
        ["MX Coord.", "MY Coord.", "MZ Coord."]
    ].mean()
    return np.array(points_avg.values)


def get_data(robot_cloud_path):
    # Robot Measurement Points, output of CMM
    W = parse_data(robot_cloud_path)
    midpoint = len(W) // 2
    W1 = W[:midpoint]
    W2 = W[midpoint:]

    return (W1, W2)


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(description="Find world-to-robot transformations.")
    parser.add_argument(
        "-r",
        "--robot_cloud_path",
        default=os.path.join(script_path, "../CMM_data/IterativePointCloud/cubes.csv"),
        help="The path to the csv file containing the calibration point clouds for each robot",
    )
    parser.add_argument("-p", "--plot", default=False)
    args = parser.parse_args()

    # robot1 (W1), and robot2 (W2) point clouds
    W1, W2 = get_data(args.robot_cloud_path)

    # Previous transformations
    T_Wold1_base1 = SE3.Rt(
        SO3.RPY([0.001234, -0.002402, -1.523872]), [0.0, 0.707012, 0.0]
    )
    T_Wold2_base2 = SE3.Rt(
        SO3.RPY([0.001232, 0.004382, 1.544656]), [0.0, -0.707012, 0.0]
    )

    T_Wold1_base1.t *= 1000  # mm
    T_Wold2_base2.t *= 1000  # mm

    # error statistics
    point_cloud_trans = W1 - W2
    point_cloud_error = np.linalg.norm(point_cloud_trans, axis=1)
    print(f"Point cloud average error (mm): {np.mean(point_cloud_error)}")
    print(f"Point cloud stddev error (mm): {np.std(point_cloud_error)}")
    print(f"Point cloud norm error (mm): {np.linalg.norm(point_cloud_error)}\n")

    # Transformation between point clouds
    T_Wold1_Wold2 = ls_registration(W1, W2)
    T_Wold1_Wnew = T_Wold1_Wold2.interp1(0.5)

    # Updated baseframe locations
    T_Wnew_base1 = T_Wold1_Wnew.inv() * T_Wold1_base1
    T_Wnew_base2 = T_Wold1_Wnew.inv() * T_Wold1_Wold2 * T_Wold2_base2

    # print("============ Updated Baseframe Transformations ============")
    # print(f"T_Wnew_base1:\n {T_Wnew_base1}")
    # print(f"T_Wnew_base2:\n {T_Wnew_base2}")
    #
    # print(f"rob1 translation (m): {T_Wnew_base1.t / 1000}")
    # print(f"rob1 orientation (rpy): {T_Wnew_base1.rpy()}")
    # print(f"rob2 translation (m): {T_Wnew_base2.t / 1000}")
    # print(f"rob2 orientation (rpy): {T_Wnew_base2.rpy()}\n")

    np.set_printoptions(suppress=True, formatter={"float_kind": "{:0.6f}".format})
    print("============ Base to World Transformations ============")
    T_base1_base2 = T_Wnew_base1.inv() * T_Wnew_base2
    T_world_base_rob1, T_world_base_rob2 = create_new_world_frame(T_base1_base2)

    print(f"T_world_base_rob1:\n {T_world_base_rob1}")
    print(f"T_world_base_rob2:\n {T_world_base_rob2}")

    print(f"rob1 translation (m): {T_world_base_rob1.t}")
    print(f"rob1 orientation (rpy): {T_world_base_rob1.rpy()}")
    print(f"rob2 translation (m): {T_world_base_rob2.t}")
    print(f"rob2 orientation (rpy): {T_world_base_rob2.rpy()}\n")

    # plot
    if args.plot:
        ax1 = plt.figure("Point clouds").add_subplot(projection="3d")
        ax1.scatter(W1[:, 0], W1[:, 1], W1[:, 2])
        ax1.scatter(W2[:, 0], W2[:, 1], W2[:, 2])
        plt.gca().set_aspect("equal")

        ax2 = plt.figure("Error distribution").add_subplot()
        ax2.bar(
            [f"point {i}" for i in range(1, len(point_cloud_error) + 1)],
            height=point_cloud_error,
        )
        ax2.set_title("Point Cloud Error Magnitude")
        ax2.set_ylabel("Error norm (mm)")
        plt.show()


if __name__ == "__main__":
    main()
