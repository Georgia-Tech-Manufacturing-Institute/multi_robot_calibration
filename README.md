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

A few dependencies need to be installed to use the Python scripts. Navigate to
the `python` directory and run the following command.
```
pip3 install -r requirements.txt
```

Test the installation with
```
python3 calibrate.py -h
```

## Data Collection

## Usage

By default, the script references the latest data stored in `CMM_data`.
To use other point clouds, pass them via the `-r` and `-t` options shown below. 

```
usage: calibrate.py [-h] [-r ROBOT_CLOUD_PATH] [-t TOOL_CLOUD_PATH]

Find world-to-robot transformations.

options:
  -h, --help            show this help message and exit
  -r ROBOT_CLOUD_PATH, --robot_cloud_path ROBOT_CLOUD_PATH
                        The path to the csv file containing the calibration point clouds for each robot
  -t TOOL_CLOUD_PATH, --tool_cloud_path TOOL_CLOUD_PATH
                        The path to the csv file containing the point cloud for the tool
```
