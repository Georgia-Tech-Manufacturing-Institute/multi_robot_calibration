import unittest

import os
import numpy as np
import pandas as pd

from roboticstoolbox import Robot
from spatialmath import SO3, SE3

from mr_calibration.core import ls_registration


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


class TestCalibration(unittest.TestCase):
    def test_ls_registration(self):
        A = np.array(
            [
                [0.3, 0.2, 0.4],
                [5.0, 5.4, 13.2],
                [4.5, 67.3, 22.0],
                [56.9, 64.0, 5.0],
                [76.0, 3.0, 43.0],
                [17.4, 51.3, 4.2],
            ]
        )
        T = SE3.Rt(SO3.Ry(0.314), [0.1, 4.0, -3.2])
        B = (T * A.T).T

        T_approx = ls_registration(A, B)
        self.assertEqual(T_approx, T)

    def test_calibration(self):
        """
        Fake world configuration:
        T_world_cmm = SE3.Rt(SO3.Rx(0.1), [0.020, 0.060, -0.060])
        T_world_base_rob1 = SE3.Rt(SO3.Rz(-90, unit="deg"), [0.0, radius, 0.0])
        T_world_base_rob2 = SE3.Rt(SO3.Rz(90, unit="deg"), [0.0, -radius, 0.0])
        """
        script_path = os.path.dirname(os.path.realpath(__file__))
        tool_cloud_path = os.path.join(script_path, "./data/tool_cloud.csv")
        robot_cloud_path = os.path.join(script_path, "./data/robot_cloud.csv")
        xacro_path = os.path.join(
            script_path, "../../robot_data/za_description/urdf/za.xacro"
        )

        V, W1, W2 = get_data(tool_cloud_path, robot_cloud_path)

        # Transformation from CMM to tool
        T_cmm_tool_rob1 = ls_registration(V, W1)
        T_cmm_tool_rob2 = ls_registration(V, W2)

        print(T_cmm_tool_rob1)

        # create robot
        robot = Robot.URDF(xacro_path)

        # Transformation from CMM to base
        q = np.deg2rad([-15, 50, 30, 80, 90, -180])

        T_tool_base_rob1 = robot.fkine(end="flange", q=q).inv()  # type: ignore
        T_tool_base_rob2 = robot.fkine(end="flange", q=q).inv()  # type: ignore
        T_cmm_base_rob1 = T_cmm_tool_rob1 * T_tool_base_rob1
        T_cmm_base_rob2 = T_cmm_tool_rob2 * T_tool_base_rob2

        # base to base transformation
        T_base_rob1_base_rob2 = T_cmm_base_rob1.inv() * T_cmm_base_rob2

        # ground truth (simulated transformation)
        radius = 0.75
        T_base_rob1_base_rob2_gt = SE3.Rt(SO3.Rz(np.pi), [2 * radius, 0, 0])

        self.assertEqual(T_base_rob1_base_rob2_gt, T_base_rob1_base_rob2)


if __name__ == "__main__":
    unittest.main()
