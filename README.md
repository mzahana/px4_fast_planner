# px4_fast_planner
Integration of [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) with [PX4](https://github.com/PX4/Firmware) for planning real-time collision-free and obstacle-free trajectories in bounded environment. This allows you to fly a multi-rotor drone ( equipped with a PX4 autopilot, depth camera, and on-board computer) autonomously while avoiding obstacles.

This packges provides installation and launch files required for running PX4 with Fast-Planner through [mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers.git) package. A modified vresion of [Fast-Planner](https://github.com/mzahana/Fast-Planner) (for Ubuntu 18 + ROS Melodic) is used.

Video:

[![px4_fast_planner](https://img.youtube.com/vi/KXXjLYjIxD0/0.jpg)](https://youtu.be/KXXjLYjIxD0 "px4_fast_planner")

**#######**

**If this helps you in your project(s), please give a star to this
 repository and [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) and [mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers). Thank you!**
 
 **#######**

# Installation
This setup is tested on Ubuntu 18 + ROS Melodic

* Clone this package into your `~/catkin_ws/src`
```sh
cd ~/catkin_ws/src
git clone https://github.com/mzahana/px4_fast_planner.git
```

* You can use the `setup.sh` script in the `install` folder to easily setup all dependencies as follows
```sh
cd ~/catkin_ws/src/px4_fast_planner/install
./setup.sh
```
The script will ask if PX4 is to be installled or not. You would likely want to skip this if you are doing the setup on an on-board computer. To do that, just pass `n` to the `setup.sh` script when asked
```sh
cd ~/catkin_ws/src/px4_fast_planner/install
./setup.sh
```
**NOTE: you may need to provide credentials to `sudo` when prompted, for the script to continue**

The `setup.sh` script install PX4 v1.10.1, `mavros_controllers` package, and modified `Fast-Planner` package.

## Installation inside docker container
You can setup the system inside a docker container that already has Ubuntu 18 + ROS Melodic + PX4 frimware v1.10.1. Use [this repository](https://github.com/mzahana/containers) to setup docker and required container, then install setup px4_fast_planner as mentioned above.

# Running in Simulation
* Open a termianl, and run the following command,
```sh
roslaunch px4_fast_planner px4_fast_planner.launch
```
**NOTE: It may take some time to download some Gazebo world models in the first time you run the simulation**

* To command the drone to fly to a target pose, publish a single message to the `/move_base_simple/goal` topic as follows
```sh
rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 19.0
    y: 15.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

# Notes
*TO ADD*
* Some notes related to tuning parameters of the availabe planners
* Some notes related to the limitations of the available planners
* Instructions to run the setup on real drone
