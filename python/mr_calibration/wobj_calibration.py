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

# robot_to_world
# three points (origin, x, y)

# 1) form plane in robot frame with three points
#    a) plane = (normal, origin)
#    b) origin = origin, normal = x.cross(y)
# 2) transform plane norm and origin to world frame

import os
import yaml
import argparse
import numpy as np

from spatialmath import SE3, SO3


def parse_data(data_path):
    with open(data_path, "r") as file:
        data = yaml.safe_load(file)

    positions = data.get("positions", [])
    xyz = data["transformation"]["xyz"]
    rpy = data["transformation"]["rpy"]

    trans = SE3.Rt(SO3.RPY(*rpy), xyz)
    return np.array(positions), trans


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(
        description="Find the world-to-work_object transformation"
    )
    parser.add_argument(
        "-d",
        "--data_path",
        default=os.path.join(script_path, "data/wobj_data.yaml"),
        help="The path to the yaml file containing the work object calibration data",
    )
    args = parser.parse_args()

    positions, T_world_base = parse_data(args.data_path)

    # Define plane in robot base
    origin, p1, p2 = positions
    x_axis = p1 - origin
    y_axis = p2 - origin
    normal = np.cross(x_axis, y_axis)

    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    normal = normal / np.linalg.norm(normal)
    T_base_wobj = SE3.Rt(np.array([x_axis, y_axis, normal]).T, origin, check=False)

    # Transform to world frame
    T_world_wobj = T_world_base * T_base_wobj

    np.set_printoptions(suppress=True, formatter={"float_kind": "{:0.6f}".format})
    print("============ Work Object Transformation ============")
    print(f"T_world_wobj:\n {T_world_wobj}")

    print(f"translation (m): {T_world_wobj.t / 1000}")
    print(f"orientation (rpy): {T_world_wobj.rpy()}")


if __name__ == "__main__":
    main()
