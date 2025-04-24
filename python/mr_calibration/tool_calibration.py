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
import pandas as pd
import numpy as np
from spatialmath import SE3

from mr_calibration.core import Robot, tool_calibration


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
    data = pd.read_csv(data_path, header=None)
    return data.to_numpy()


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(
        description="Find the flange-to-tool transformation"
    )
    parser.add_argument(
        "-q",
        "--joint_configs_path",
        default=os.path.join(script_path, "data/tool_calib_configs.csv"),
        help="The path to the csv file containing the calibration point clouds for each robot",
    )
    parser.add_argument(
        "-d",
        "--deg",
        default=False,
        help="Pass this argument if the data is in degrees",
    )
    args = parser.parse_args()

    # Read joint configurations
    qq = parse_data(args.joint_configs_path)
    if args.deg:
        qq = np.deg2rad(qq)

    # create robot
    robot = Robot(za6())
    robot.tool = SE3.Rx(np.pi) * SE3.Ry(np.pi / 2)

    initial_guess = np.array([0.024, 0.00186, -0.108117]) * 1000  # mm
    res = tool_calibration(robot, qq, initial_guess)

    np.set_printoptions(suppress=True, formatter={"float_kind": "{:0.6f}".format})
    print(f"Tool translation (mm): {res}")
    print(f"Tool translation (m): {res / 1000}")


if __name__ == "__main__":
    main()
