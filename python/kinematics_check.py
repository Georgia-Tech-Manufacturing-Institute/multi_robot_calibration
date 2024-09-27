import os
import argparse

from mr_calibration.geometry import ls_registration, three_point_circle
from mr_calibration.keyence_cmm import read_point_cloud


def get_data(tool_cloud_path, robot_cloud_path):
    # Designed points in tool frame
    V = read_point_cloud(tool_cloud_path)

    # Robot Measurement Points, output of CMM
    W = read_point_cloud(robot_cloud_path)
    W1 = W[:20]
    W2 = W[20:40]
    W3 = W[40:60]

    return (V, W1, W2, W3)


def main():
    script_path = os.path.dirname(os.path.realpath(__file__))

    # Parse input csv file locations
    parser = argparse.ArgumentParser(description="Find world-to-robot transformations.")
    parser.add_argument(
        "-r",
        "--robot_cloud_path",
        help="The path to the csv file containing the (60pt) point cloud for a robot",
    )
    parser.add_argument(
        "-t",
        "--tool_cloud_path",
        default=os.path.join(script_path, "../CMM_data/ToolCloud/tool_cloud.csv"),
        help="The path to the csv file containing the point cloud for the tool",
    )
    args = parser.parse_args()

    V, W1, W2, W3 = get_data(args.tool_cloud_path, args.robot_cloud_path)

    T_cmm_tool_1 = ls_registration(V, W1)
    T_cmm_tool_2 = ls_registration(V, W2)
    T_cmm_tool_3 = ls_registration(V, W3)

    c, r = three_point_circle(T_cmm_tool_1.t, T_cmm_tool_2.t, T_cmm_tool_3.t)
    print(f"Circle center: {c} mm")
    print(f"Circle radius: {r} mm")

    # plot
    import matplotlib.pyplot as plt

    T_cmm_tool_1.plot()
    T_cmm_tool_2.plot()
    T_cmm_tool_3.plot()
    plt.gca().set_aspect("equal")
    plt.show()


if __name__ == "__main__":
    main()
