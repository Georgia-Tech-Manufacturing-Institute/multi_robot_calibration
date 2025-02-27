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

1) `calibrate-bf` for base-frame calibration using closed form transformations
2) `calibrate-iterative` for iterative multi-robot calibration

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
usage: calibrate-iterative [-h] [-r ROBOT_CLOUD_PATH] [-p | --plot | --no-plot]

Find world-to-robot transformations.

options:
  -h, --help            show this help message and exit
  -r ROBOT_CLOUD_PATH, --robot_cloud_path ROBOT_CLOUD_PATH
                        The path to the csv file containing the calibration point clouds for each robot
  -p, --plot, --no-plot
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
