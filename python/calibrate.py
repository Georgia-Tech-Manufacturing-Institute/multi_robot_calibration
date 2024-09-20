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
from spatialmath import SO3, SE3
from roboticstoolbox import Robot

from mr_calibration import ls_registration


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
    V /= 1000

    # Robot Measurement Points, output of CMM
    W = parse_data(robot_cloud_path)
    midpoint = len(W) // 2
    W1 = W[:midpoint] / 1000
    W2 = W[midpoint:] / 1000

    return (V, W1, W2)


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(description="Find world-to-robot transformations.")
    parser.add_argument(
        "-r",
        "--robot_cloud_path",
        default=os.path.join(script_path, "../CMM_data/40PointCloudRaw/robot_to_robot_091124.csv"),
        help="The path to the csv file containing the calibration point clouds for each robot",
    )
    parser.add_argument(
        "-t",
        "--tool_cloud_path",
        default=os.path.join(script_path, "../CMM_data/ToolCloud/tool_cloud.csv"),
        help="The path to the csv file containing the point cloud for the tool",
    )
    args = parser.parse_args()

    print(args.robot_cloud_path)

    # tool (V), robot1 (W1), and robot2 (W2) point clouds
    V, W2, W1 = get_data(args.tool_cloud_path, args.robot_cloud_path)

    # create robot
    xacro_path = script_path + "/../robot_data/za_description/urdf/za.xacro"
    robot = Robot.URDF(xacro_path)

    # Transformation from CMM to tool
    T_cmm_tool_rob1 = ls_registration(V, W1)
    T_cmm_tool_rob2 = ls_registration(V, W2)

    # Transformation from CMM to base
    # q1 = np.deg2rad([0, 94.119, -6.812, 0, -87.307, 0])
    # q2 = np.deg2rad([-11.278, 53.020, 17.195, -82.596, -122.65, 48.863])
    q1 = np.deg2rad([0, 21, 45, 0, -66, 0])
    q2 = np.deg2rad([0, 21, 45, 0, -66, 0])
    T_tool_base_rob1 = robot.fkine(end="flange", q=q1).inv()  # type: ignore
    T_tool_base_rob2 = robot.fkine(end="flange", q=q2).inv()  # type: ignore
    T_cmm_base_rob1 = T_cmm_tool_rob1 * T_tool_base_rob1
    T_cmm_base_rob2 = T_cmm_tool_rob2 * T_tool_base_rob2

    # Construct world frame between robots
    t_cmm_world = 0.5 * (T_cmm_base_rob1.t + T_cmm_base_rob2.t)
    T_cmm_world = SE3.Rt(SO3().Rz(np.pi / 2), t_cmm_world)

    # convert to world frame
    T_world_base_rob1 = T_cmm_world.inv() * T_cmm_base_rob1
    T_world_base_rob2 = T_cmm_world.inv() * T_cmm_base_rob2
    print(f"T_world_base_rob1:\n {T_world_base_rob1}")
    print(f"T_world_base_rob2:\n {T_world_base_rob2}")

    print(f"rob1 translation: {T_world_base_rob1.t}")
    print(f"rob1 orientation (rpy): {T_world_base_rob1.rpy()}")
    print(f"rob2 translation: {T_world_base_rob2.t}")
    print(f"rob2 orientation (rpy): {T_world_base_rob2.rpy()}")

    # base to base transformation
    T_base_world_rob1 = T_world_base_rob1.inv()
    T_base_world_rob2 = T_world_base_rob2.inv()
    T_base_rob1_base_rob2 = T_base_world_rob1 * T_world_base_rob2
    print(T_base_rob1_base_rob2)

    # reverse transformation (pathpilot input)
    print(f"T_base_world_rob1:\n {T_base_world_rob1}")
    print(f"T_base_world_rob2:\n {T_base_world_rob2}")

    print(f"rob1 translation: {T_base_world_rob1.t * 1000}")
    print(f"rob1 orientation (rpy): {np.rad2deg(T_base_world_rob1.rpy())}")
    print(f"rob2 translation: {T_base_world_rob2.t * 1000}")
    print(f"rob2 orientation (rpy): {np.rad2deg(T_base_world_rob2.rpy())}")

    # plot
    import matplotlib.pyplot as plt

    T_world_base_rob1.plot(frame="B1")
    T_world_base_rob2.plot(frame="B2")
    plt.show()


if __name__ == "__main__":
    main()
