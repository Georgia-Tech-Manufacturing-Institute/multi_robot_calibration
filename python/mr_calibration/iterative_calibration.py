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
import yaml
import numpy as np
import pandas as pd
from typing import Dict
from spatialmath import SO3, SE3

import matplotlib.pyplot as plt
from mr_calibration.core.geometry import ls_registration
from mr_calibration.core import create_new_world_frame

POINTS_PER_CORNER = 6


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


def create_cube(l, w, h):
    l2, w2, h2 = l / 2, w / 2, h / 2
    points = np.array(
        [
            [l2, w2, h2],
            [l2, -w2, h2],
            [-l2, -w2, h2],
            [-l2, w2, h2],
            [-l2, w2, -h2],
            [l2, w2, -h2],
            [l2, -w2, -h2],
            [-l2, -w2, -h2],
        ]
    )
    return points


def average_every_m_points(mat, M):
    N = mat.shape[0]
    if N % M != 0:
        raise ValueError(
            f"The number of rows N must be divisible by M, but N = {N} and M = {M}"
        )

    reshaped_arr = mat.reshape(N // M, M, 3)
    averaged_arr = reshaped_arr.mean(axis=1)
    return averaged_arr


def yaml_to_transform(yaml_dict: Dict) -> SE3:
    return SE3.Rt(
        SO3.RPY([yaml_dict["roll"], yaml_dict["pitch"], yaml_dict["yaw"]]),
        [yaml_dict["x"], yaml_dict["y"], yaml_dict["z"]],
    )


def transform_to_yaml(transform: SE3) -> Dict:
    xyz = [float(x) for x in transform.t]
    rpy = [float(x) for x in transform.rpy()]
    return {
        "x": xyz[0],
        "y": xyz[1],
        "z": xyz[2],
        "roll": rpy[0],
        "pitch": rpy[1],
        "yaw": rpy[2],
    }


def get_default_output_path(input_path):
    dir_name = os.path.dirname(input_path)
    base_name = os.path.basename(input_path)
    name, ext = os.path.splitext(base_name)
    modified_name = f"{name}_modified{ext}"
    return os.path.join(dir_name, modified_name)


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(description="Find world-to-robot transformations.")
    parser.add_argument(
        "-r",
        "--robot_cloud_path",
        default=os.path.join(script_path, "data/cubes.csv"),
        help="The path to the csv file containing the calibration point clouds for each robot",
    )
    parser.add_argument(
        "-b",
        "--base_frames_path",
        default=os.path.join(script_path, "data/base_frames.yaml"),
        help="The path to the yaml file containing the previous base frame transformations",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Optional output filename for the modified base frames",
    )
    parser.add_argument("-p", "--plot", action=argparse.BooleanOptionalAction)
    args = parser.parse_args()

    # robot1 (W1), and robot2 (W2) point clouds
    W1, W2 = get_data(args.robot_cloud_path)

    # read previous transformations
    with open(args.base_frames_path, "r") as file:
        frames = yaml.safe_load(file)

    T_Wold1_base1 = yaml_to_transform(frames["rob1_base"])
    T_Wold2_base2 = yaml_to_transform(frames["rob2_base"])

    T_Wold1_base1.t *= 1000  # mm
    T_Wold2_base2.t *= 1000  # mm

    # error statistics
    point_cloud_trans = W1 - W2
    point_cloud_error = np.linalg.norm(point_cloud_trans, axis=1)
    print(f"Point cloud average error (mm): {np.mean(point_cloud_error)}")
    print(f"Point cloud stddev error (mm): {np.std(point_cloud_error)}")
    print(f"Point cloud norm error (mm): {np.linalg.norm(point_cloud_error)}")
    print(f"Point cloud sum error (mm) {np.sum(point_cloud_error)}\n")

    # Transformation between point clouds
    points = create_cube(200, 200, 200) + np.array([0.0, 0.0, 200])
    W1_avg = average_every_m_points(W1, 6)
    W2_avg = average_every_m_points(W2, 6)

    T_Wold1_CMM = ls_registration(W1_avg, points)
    T_CMM_Wold2 = ls_registration(points, W2_avg)

    T_Wold1_Wold2 = T_Wold1_CMM * T_CMM_Wold2
    T_Wold1_Wnew = T_Wold1_Wold2.interp1(0.5)

    # Updated baseframe locations
    T_Wnew_base1 = T_Wold1_Wnew.inv() @ T_Wold1_base1
    T_Wnew_base2 = T_Wold1_Wnew.inv() @ T_Wold1_Wold2 @ T_Wold2_base2

    np.set_printoptions(suppress=True, formatter={"float_kind": "{:0.6f}".format})
    print("============ Base to World Transformations ============")
    T_base1_base2 = T_Wnew_base1.inv() * T_Wnew_base2
    T_world_base_rob1, T_world_base_rob2 = create_new_world_frame(T_base1_base2)

    print(f"T_world_base_rob1:\n {T_world_base_rob1}")
    print(f"T_world_base_rob2:\n {T_world_base_rob2}")

    T_world_base_rob1.t /= 1000  # m
    T_world_base_rob2.t /= 1000  # m

    # Write updated frames to yaml
    output_path = args.output or get_default_output_path(args.base_frames_path)
    output = {
        "rob1_base": transform_to_yaml(T_world_base_rob1),
        "rob2_base": transform_to_yaml(T_world_base_rob2),
    }

    def float_representer(dumper, value):
        text = f"{value:.6f}"
        return dumper.represent_scalar("tag:yaml.org,2002:float", text)

    yaml.add_representer(float, float_representer)

    with open(output_path, "w") as outfile:
        yaml.dump(output, outfile, sort_keys=False)

    print(f"Updated base frame parameters output to: {output_path}")

    # plot
    if args.plot:
        ax1 = plt.figure("Point clouds").add_subplot(projection="3d")
        ax1.scatter(W1[:, 0], W1[:, 1], W1[:, 2])
        ax1.scatter(W2[:, 0], W2[:, 1], W2[:, 2])
        plt.gca().set_aspect("equal")

        ax2 = plt.figure("Error distribution").add_subplot()
        for i in range(int(len(point_cloud_error) / POINTS_PER_CORNER)):
            xs, xe = POINTS_PER_CORNER * i, POINTS_PER_CORNER * (i + 1)
            ax2.bar(
                x=range(xs, xe),
                height=point_cloud_error[xs:xe],
            )
        ax2.set_title("Point Cloud Error Magnitude")
        ax2.set_ylabel("Error norm (mm)")
        plt.show()


if __name__ == "__main__":
    main()
