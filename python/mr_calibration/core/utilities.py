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

from typing import Dict
from spatialmath import SO3, SE3


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


def float_representer(dumper, value):
    text = f"{value:.6f}"
    return dumper.represent_scalar("tag:yaml.org,2002:float", text)
