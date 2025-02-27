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
from pandas import read_csv


def read_point_cloud(data_path):
    """
    Read a csv point cloud as a numpy array.

    Returns an averaged point cloud from the output of the Leica CMM
    'Measurement Point List'. The csv is assumed to have headers:

    Element Name, Number, Deviation, MX Coord., MY Coord., MZ Coord.

    Rows with identical names are averaged.

    Parameters
    ----------
    data_path : string
        The path to the csv point cloud data

    Returns
    -------
    point_cloud : ndarray
        The averaged point cloud
    """
    data = read_csv(data_path)
    points_avg = data.groupby("Element Name")[
        ["MX Coord.", "MY Coord.", "MZ Coord."]
    ].mean()
    return np.array(points_avg.values)
