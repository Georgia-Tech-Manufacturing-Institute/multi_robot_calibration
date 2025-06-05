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
# three points (origin, p1, p2)

# 1) form transformation in robot frame with three points
#    b) x-axis = unit(p1 - origin)
#    c) z-axis = unit(x-axis.cross(p2 - origin))
#    d) y-axis = unit(z-axis.cross(x-axis))
#    e) R = [x-axis, y-axis, z-axis]
# 2) transform R and origin to world frame

import os
import yaml
import argparse
import numpy as np

from spatialmath import SE3
from mr_calibration.core.utilities import *


def parse_data(data_path):
    with open(data_path, "r") as file:
        data = yaml.safe_load(file)

    positions = data.get("positions", [])
    trans = yaml_to_transform(data["transformation"])
    return np.array(positions), trans


def get_default_output_path(input_path):
    dir_name = os.path.dirname(input_path)
    output_name = f"calibrated_wobj.yaml"
    return os.path.join(dir_name, output_name)


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
    parser.add_argument(
        "-o",
        "--output",
        help="Optional output filename for the wobj frame",
    )
    args = parser.parse_args()

    positions, T_world_base = parse_data(args.data_path)

    # Define plane in robot base
    origin, p1, p2 = positions
    x_axis = p1 - origin
    z_axis = np.cross(x_axis, p2 - origin)
    y_axis = np.cross(z_axis, x_axis)

    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    T_base_wobj = SE3.Rt(np.array([x_axis, y_axis, z_axis]).T, origin)

    # Transform to world frame
    T_world_wobj = T_world_base @ T_base_wobj

    np.set_printoptions(suppress=True, formatter={"float_kind": "{:0.6f}".format})
    print("============ Work Object Transformation ============")
    print(f"T_world_wobj:\n {T_world_wobj}")

    # Write updated wobj to yaml
    output_path = args.output or get_default_output_path(args.data_path)
    output = {
        "work_object": transform_to_yaml(T_world_wobj),
    }

    yaml.add_representer(float, float_representer)
    with open(output_path, "w") as outfile:
        yaml.dump(output, outfile, sort_keys=False)

    print(f"Updated base frame parameters output to: {output_path}")


if __name__ == "__main__":
    main()
