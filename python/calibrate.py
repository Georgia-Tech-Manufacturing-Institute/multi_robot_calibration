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
import pandas as pd
import numpy as np
import argparse
import matplotlib.pyplot as plt
from spatialmath import SE3

from mr_calibration import Robot
from mr_calibration import ls_registration
from mr_calibration.core import create_new_world_frame


def za6():
    return np.array(
        [
            [-np.pi / 2, 25, 0, 450],
            [0, 454, -np.pi / 2, -1],
            [-np.pi / 2, 35, 0, 0],
            [np.pi / 2, 0, 0, 419.5],
            [-np.pi / 2, 0, 0, 1],
            [0, 0, 0, 117.5],
        ]
    )


def parse_data(data_path):
    """Returns averaged point cloud"""
    data = pd.read_csv(data_path)
    points_avg = data.groupby("Element Name")[
        ["MX Coord.", "MY Coord.", "MZ Coord."]
    ].mean()
    return np.array(points_avg.values)


def get_data(tool_cloud_path, robot_cloud_path):
    # Designed points in tool frame
    V = parse_data(tool_cloud_path)

    # Robot Measurement Points, output of CMM
    W = parse_data(robot_cloud_path)
    midpoint = len(W) // 2
    W1 = W[:midpoint]
    W2 = W[midpoint:]

    return (V, W1, W2)


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(description="Find world-to-robot transformations.")
    parser.add_argument(
        "-r",
        "--robot_cloud_path",
        default=os.path.join(
            script_path, "../CMM_data/40PointCloudRaw/rob2_self_transform_092024.csv"
        ),
        help="The path to the csv file containing the calibration point clouds for each robot",
    )
    parser.add_argument(
        "-t",
        "--tool_cloud_path",
        default=os.path.join(script_path, "../CMM_data/ToolCloud/tool_cloud.csv"),
        help="The path to the csv file containing the point cloud for the tool",
    )
    parser.add_argument("-p", "--plot", default=False)
    args = parser.parse_args()

    # tool (V), robot1 (W1), and robot2 (W2) point clouds
    V, W1, W2 = get_data(args.tool_cloud_path, args.robot_cloud_path)

    # create robot
    robot = Robot(za6())
    robot.tool = SE3.Rx(np.pi) * SE3.Ry(np.pi / 2)

    # Transformation from CMM to tool
    T_cmm_tool_rob1 = ls_registration(V, W1)
    T_cmm_tool_rob2 = ls_registration(V, W2)

    # Transformation from CMM to base
    # rob2_self_transform_092024.csv
    q1 = np.deg2rad([0.137, 38.415, 52.065, 1.298, -90.610, 14.454])
    q2 = np.deg2rad([26.406, 23.369, 9.66, 105.939, -67.422, -35.926])

    T_tool_base_rob1 = robot.fk(q1).inv()  # type: ignore
    T_tool_base_rob2 = robot.fk(q2).inv()  # type: ignore
    T_cmm_base_rob1 = T_cmm_tool_rob1 * T_tool_base_rob1
    T_cmm_base_rob2 = T_cmm_tool_rob2 * T_tool_base_rob2

    # base to base transformation
    T_base_rob1_base_rob2 = T_cmm_base_rob1.inv() * T_cmm_base_rob2
    print("============ Base to Base Transformation ============")
    print(f"T_base_rob1_base_rob2:\n {T_base_rob1_base_rob2}")
    print(f"Origin distance (mm): {np.linalg.norm(T_base_rob1_base_rob2.t)}\n")

    # convert to world frame
    T_world_base_rob1, T_world_base_rob2 = create_new_world_frame(T_base_rob1_base_rob2)

    np.set_printoptions(suppress=True, formatter={"float_kind": "{:0.6f}".format})
    print("============ Base to World Transformations ============")
    print(f"T_world_base_rob1:\n {T_world_base_rob1}")
    print(f"T_world_base_rob2:\n {T_world_base_rob2}")

    print(f"rob1 translation (m): {T_world_base_rob1.t / 1000}")
    print(f"rob1 orientation (rpy): {T_world_base_rob1.rpy()}")
    print(f"rob2 translation (m): {T_world_base_rob2.t / 1000}")
    print(f"rob2 orientation (rpy): {T_world_base_rob2.rpy()}\n")

    # plot
    if args.plot:
        T_world_base_rob1.plot(frame="B1")
        T_world_base_rob2.plot(frame="B2")
        plt.gca().set_aspect("equal")
        plt.show()


if __name__ == "__main__":
    main()
