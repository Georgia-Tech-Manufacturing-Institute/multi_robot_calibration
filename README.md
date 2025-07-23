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

##### AMPF Usage:

Obtain the Coordinate Measuring Machine, and place it upon the tripod pedastal such that it is centered between the machines, and offset away from the base to base center line. The scanner should have equal line of sight to both end effectors. The Tormach Arms should both be positioned in the same orientation, using joint angles [0, 21, 45, 0, -66, 0]. The handheld probe should be using the 5mm tip.

Turn on the scanner, the accompanying laptop, and the handheld probe. Start the CMM software and select the single points scanning mode. Select the "Point" option for scanning. You will be probing the unfilled bolt holes on the faces of each arm's end effector, starting with the top left and continuing clockwise until all 6 have been scanned. You will start with the right hand robot when observing the arms from the scanner's perspective. Touch the probe to the opening of the top left empty bolt hole (the light on the Scanner should be green. If orange/yellow is the only color to obtain the necessary scan, that is acceptable but less desireable) and pull the trigger. A sound will play indicating successful scan. Collect 3-4 data points per bolt hole before pressing done on the laptop. This will be "Point 1" in the list of scanned objects on the left of the CMM software. Repeat this process for the next 5 empty bolt holes on the first end effector, and then repeat the process on the second end effector.

In the toolbar for the CMM software there will be an option to export scanned points list. The option that outputs all probed points in a csv format is desired. Place that file in the directory of your choosing, and run the python script with the proper path specified in the options.

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
##### AMPF Usage:

Obtain the Coordinate Measuring Machine, and place it upon the tripod pedastal such that it is centered between the machines, and offset away from the base to base center line. The scanner should have equal line of sight to both end effectors. The handheld probe should be using the 5mm tip.

This procedure involves probing the six empty bolt holes of the end effector of each arm at the 8 points that form a cube in working area of the arms. You will run a script on the Tormach control desktop that moves the arms through the points of the cube, with movement from one point to the next controlled by a user's input throught the keyboard. The first robot will walk throught the points, then the second robot will walk through the points. The goal is for the points that each arm targets to be the same points in space. This will not be the case during the first iteration, so 3-4 cycles of scanning and iterating are required.

The following commands are required to put the arms into this usage mode. All commands should be run in the terminal of the specified computer. The three computers are the main host and the 2 arm computers.

1. Run Roscore in the terminal on the host
2. On each arm computer (after powering on and activating both arms):
  * cd ~/dev
  * cd za_docker/
  * ./tormach_ros_dev_container.sh -x
  * cd ../
  * cd ros_workspaces/
  * cd hydra
  * source  devel/setup.bash
  * roslaunch hydra_bringup hydra_robot.launch amr_id:=rob1 hardware:=hal sim:=false
    + For the first arm hardware:=rob1, for the second hardware:=rob2
3. Return to host computer
  * Open a new terminal
  * "insert command to start the calibration script"
4. Once the 6 empty bolt holes are scanned at a vertex of the cube, provide an input to the host computer to reach the next vertext, and repeat for each vertex and each arm
5. You will end with 96 scanned point objects. 6 objects (boltholes) per vertex, 8 vertexes per cube, with 2 cubes. Refer to the previous section for directions on scanning points on the end effectors.
6. Export the CSV point list and run the `calibrate-iterative` script upon it
  * The first initial base calibration frames should be found via the steps under the `calibrate-bf`
7. Place the outputted frames in the proper locations on the robots
8. Repeat until the errors are acceptable for your usecase. Generally 0.5mm-0.6mm of average error is good enough to stop.

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
