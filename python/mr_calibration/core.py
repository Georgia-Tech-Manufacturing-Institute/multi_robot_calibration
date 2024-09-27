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

from roboticstoolbox import Robot
from spatialmath import SO3, SE3

from mr_calibration.geometry import ls_registration


def calibrate_to_world(
    tool_cloud, robot_clouds, robot_configs, tool_frames, xacro_path
):
    # read robot (for now we assume all robots are identical)
    robot = Robot.URDF(xacro_path)

    # transformations from CMM to tool
    T_cmm_tool = []
    for i, r_cloud in enumerate(robot_clouds):
        try:
            T_cmm_tool_r = ls_registration(tool_cloud, r_cloud)
            T_cmm_tool.append(T_cmm_tool_r)
        except ValueError:
            print(f"Invalid point cloud registration for robot {i}")
            return None

    # transformations from CMM to base
