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

from copy import deepcopy
from enum import Enum
from pandas import DataFrame
from spatialmath import SE3


def dh_fk(dh_params, q):
    T = SE3()
    for dh, qn in zip(dh_params, q):
        cαn, sαn = np.cos(dh[0]), np.sin(dh[0])
        an = dh[1]
        cθn, sθn = np.cos(dh[2] + qn), np.sin(dh[2] + qn)
        dn = dh[3]

        T *= SE3(
            np.array(
                [
                    [cθn, -sθn * cαn, sθn * sαn, an * cθn],
                    [sθn, cθn * cαn, -cθn * sαn, an * sθn],
                    [0, sαn, cαn, dn],
                    [0, 0, 0, 1],
                ]
            )
        )
    return T


def mdh_fk(dh_params, q):
    T = SE3()
    for dh, qn in zip(dh_params, q):
        cαn_1, sαn_1 = np.cos(dh[0]), np.sin(dh[0])
        an_1 = dh[1]
        cθn, sθn = np.cos(dh[2] + qn), np.sin(dh[2] + qn)
        dn = dh[3]

        T *= SE3(
            np.array(
                [
                    [cθn, -sθn, 0, an_1],
                    [sθn * cαn_1, cθn * cαn_1, -sαn_1, -dn * sαn_1],
                    [sθn * sαn_1, cθn * sαn_1, cαn_1, dn * cαn_1],
                    [0, 0, 0, 1],
                ]
            )
        )
    return T


class DHType(Enum):
    STANDARD = dh_fk
    MODIFIED = mdh_fk


class Robot(object):
    def __init__(self, dh_params, tool=SE3(), dh_type=DHType.STANDARD):
        self._dh_params = dh_params
        self.tool = tool  # T_link[-1]_tool
        self._fk = dh_type

    @property
    def dh_params(self):
        return self._dh_params

    @dh_params.setter
    def dh_params(self, value):
        self._dh_params = value

    @property
    def njoints(self):
        return len(self._dh_params)

    @property
    def tool(self):
        return self._tool

    @tool.setter
    def tool(self, value: SE3):
        self._tool = value

    def __str__(self):
        df = DataFrame(self._dh_params)
        df.columns = ["alpha", "a", "theta", "d"]
        return str(df) + "\n"

    def copy(self):
        return deepcopy(self)

    def fk(self, q) -> SE3:
        q = np.atleast_2d(q)
        return SE3([self._fk(self._dh_params, qn) for qn in q]) @ self.tool


if __name__ == "__main__":

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

    robot = Robot(puma560())
    # robot.tool = SE3.Rx(np.pi) * SE3.Ry(np.pi / 2)

    print(robot)

    q = [1, 1, 1, 1, 1, 1]
    q2 = [0, 0, 0, 0, 0, 0]

    qarray = np.array([q, q2])
    print(robot.fk(qarray))
