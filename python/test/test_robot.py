import unittest

import numpy as np
from spatialmath import SE3

from mr_calibration import Robot, DHType


def puma560():
    return np.array(
        [
            [np.pi / 2, 0, 0, 0],
            [0, 0.4318, 0, 0],
            [-np.pi / 2, 0.0203, 0, 0.15005],
            [np.pi / 2, 0, 0, 0.4318],
            [-np.pi / 2, 0, 0, 0],
            [0, 0, 0, 0],
        ]
    )


class TestRobot(unittest.TestCase):
    def test_dh_forward_kinematics(self):
        robot = Robot(puma560(), dh_type=DHType.STANDARD)
        q1 = np.zeros(6)
        q2 = np.repeat(np.pi, 6)

        pose1 = SE3(0.4521, -0.15005, 0.4318)
        pose2 = SE3(0.4115, 0.15005, 0.4318) * SE3.Rx(np.pi)

        # individual configurations
        T = robot.fk(q1)
        self.assertEqual(T, pose1)

        T = robot.fk(q2)
        self.assertEqual(T, pose2)

        # list of configurations
        T = robot.fk([q1, q2])
        self.assertEqual(T, SE3([pose1, pose2]))

    def test_mdh_forward_kinematics(self):
        robot = Robot(puma560(), dh_type=DHType.MODIFIED)
        q1 = np.zeros(6)
        q2 = np.repeat(np.pi, 6)

        pose1 = SE3(0.4521, -0.4318, 0.15005)
        pose2 = SE3(-0.4115, 0.4318, 0.15005) * SE3.Rx(np.pi)

        # individual configurations
        T = robot.fk(q1)
        print(T)
        self.assertEqual(T, pose1)

        T = robot.fk(q2)
        self.assertEqual(T, pose2)

        # list of configurations
        T = robot.fk([q1, q2])
        self.assertEqual(T, SE3([pose1, pose2]))

    def test_tool(self):
        robot = Robot(puma560(), dh_type=DHType.STANDARD)
        q1 = np.zeros(6)

        T_flange_tool = SE3.Ry(np.pi / 3) * SE3.Rz(3) * SE3(1, 2, 3)
        T_base_flange = robot.fk(q1)

        robot.tool = T_flange_tool
        T_base_tool = robot.fk(q1)

        self.assertEqual(robot.tool, T_base_flange.inv() * T_base_tool)


if __name__ == "__main__":
    unittest.main()
