# Multi-Robot Calibration

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

This repository contains a collection of scripts for calibrating the
[Hydra](https://github.com/alexarbogast/hydra_ros) multi-robot system at Georgia
Tech's [Advanced Manufacturing Pilot Facility
(AMPF)](https://ampf.research.gatech.edu/). There are Python and Matlab versions
of the calibration routines.

## Installation

Clone the repository,

```
git clone git@github.com:Georgia-Tech-Manufacturing-Institute/multi_robot_calibration.git
```

The Python package and it's dependencies are installed locally with pip.

```
git clone git@github.com:Georgia-Tech-Manufacturing-Institute/multi_robot_calibration.git
cd multi_robot_calibration/python
pip install -e .
```

For developers, if you would like to run the Python unit tests.

```
pip install -e .[dev]
pytest
```

## Data Collection

## Usage

The Python installation will expose two scripts.

1. `calibrate-bf` for base-frame calibration using closed form transformations
2. `calibrate-iterative` for iterative multi-robot calibration

By default, the script references the example data stored in the `data` folder.
To use other point clouds, pass them via the `-r` and `-t` options shown below.

#### `calibrate-bf`

```
usage: calibrate-bf [-h] [-r ROBOT_CLOUD_PATH] [-t TOOL_CLOUD_PATH] [-p PLOT]

Find world-to-robot transformations.

options:
  -h, --help            show this help message and exit
  -r ROBOT_CLOUD_PATH, --robot_cloud_path ROBOT_CLOUD_PATH
                        The path to the csv file containing the calibration point clouds for each
                        robot
  -t TOOL_CLOUD_PATH, --tool_cloud_path TOOL_CLOUD_PATH
                        The path to the csv file containing the point cloud for the tool
  -p PLOT, --plot PLOT
```

#### `calibrate-iterative`

```
usage: calibrate-iterative [-h] [-r ROBOT_CLOUD_PATH] [-b BASE_FRAMES_PATH] [-o OUTPUT] [-p | --plot | --no-plot]

Find world-to-robot transformations.

options:
  -h, --help            show this help message and exit
  -r ROBOT_CLOUD_PATH, --robot_cloud_path ROBOT_CLOUD_PATH
                        The path to the csv file containing the calibration point clouds for each robot
  -b BASE_FRAMES_PATH, --base_frames_path BASE_FRAMES_PATH
                        The path to the yaml file containing the previous base frame transformations
  -o OUTPUT, --output OUTPUT
                        Optional output filename for the modified base frames
  -p, --plot, --no-plot
```

#### `calibrate-tool`

```
usage: calibrate-tool [-h] [-q JOINT_CONFIGS_PATH]

Find the flange-to-tool transformation

options:
  -h, --help            show this help message and exit
  -q JOINT_CONFIGS_PATH, --joint_configs_path JOINT_CONFIGS_PATH
                        The path to the csv file containing the calibration point clouds for each robot
```

#### `calibrate-wobj`

```
usage: calibrate-wobj [-h] [-d DATA_PATH] [-o OUTPUT]

Find the world-to-work_object transformation

options:
  -h, --help            show this help message and exit
  -d DATA_PATH, --data_path DATA_PATH
                        The path to the yaml file containing the work object calibration data
  -o OUTPUT, --output OUTPUT
                        Optional output filename for the wobj frame
```

## Testing

The Python library uses the [pytest](https://doc.pytest.org/en/latest/)
framework. First, install the dependencies required by the tests. From the root
directory of the repository,

```
cd multi_robot_calibration/python
pip install -e .[dev]
pytest
```
