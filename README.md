# Programming Assignment 1: Robot Sensors

Simple nodes for moving the Husarion ROSBot 2.0, record ROS topics, and plot robot's x and y pose.

## Getting Started

This package has been tested on Ubuntu 16.04 and on the Husarion ROSBot 2.0.

Set up your catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

## Install

Clone the repository. Then build and source:
```
git clone https://github.com/cs169-wi20-robotics-perception-systems/pa1.git
cd ..
catkin_make
source devel/setup.bash
```

Check if the `move_rosbot_forward.py` and `plot_rosbot_odom.py` files in `scripts` folder are executable. If not, run:
```
chmod +x move_rosbot_forward.py
chmod +x plot_rosbot_odom.py
```

## Run

### Part 1
To move the robot forward at set distance and linear speed:
```
roslaunch pa1 move_forward.launch [record:=0/1 distance:=value linear_speed:=value]
```

### Part 2
To log the ROS topics, which will be saved in the `data` directory of the package:
```
roslaunch pa1 logger.launch
```
If the `data` folder does not exist in the package, please create one:
```
mkdir data
```

### Part 3
To plot the robot's x and y pose, which can be visualized in `rviz` as markers from the ROS topic `rosbot8/frame`:
```
roslaunch pa1 plot_odom.launch [record:=0/1]
```
