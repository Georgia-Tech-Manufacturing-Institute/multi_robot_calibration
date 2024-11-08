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


from mr_calibration.geometry import ls_registration


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
