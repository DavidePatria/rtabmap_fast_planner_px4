# Intro

The purpose of this project is to simulate a drone navigating in an indoor environment, supplied with a visual SLAM and a path planner of choice.
The used version of px4 is `v1.12.3`

This repository contains a few ros subpackages, both tested on **noetic**.
Please refer to the readme files in the subpackages for details.

# Sources

original repositories of some components and that gave inspiration to the present project:

- *rtabmap SLAM* and example of offboard control with px4 directly [repo](`https://github.com/matlabbe/rtabmap_drone_example`)
- [fast planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) that only works on melodic
- [fork of fast planner](https://github.com/mzahana/Fast-Planner) oriented towards integration with px4 on melodic
- integration of `px4`, `fast planner` that uses [geometric controller](https://github.com/Jaeyoung-Lim/mavros_controllers), but were the controlle is not tuned properly, so the drone stumbles and ends upside down and unfortunately it only works on *melodic*

# Required packages

to setup your workspace for the simulation do the following:

- clone `https://github.com/DavidePatria/mavros_controllers` for the geometric controller, making also sure to have the required dependencies.
- clone this repo, making sure to check the single packages readmes to install the required dependencies
- build the container based on *ubuntu 18.04* and *ros melodic* to run *Fast-Planner*


# Important notes

The rtabmap_drone_example mentioned above already containes a 2D planner based on *move_base* (sorry for the alliteration), which is not enough for the purpose of this work, this is why it has been removed and another planner has been added.


# Launching simulation

The launch files needed for everything but *fast planner* are inside *rtabmap_drone*. 

# Simulation with fast planner

Firsly, to launch the gazebo simulation and spawn the drone use
`roslaunch rtabmap_drone gazebo.launch vehicle:=iris_depth world:=small_house gui:=false`
For a list of available worlds look into `./rtabmap_drone/worlds/` and pass the name of the file as an argument when roslaunching.

The *gui* option is set to `false` since the slam has rviz that can do the job of showing the position of the drone in the map, but it can also be set to true.

The above launch file and the ones called from it contain some specific setting depending on the used world file since position [0;0;0] is not always free and the drone needs to be spawned in a particular configuration.

To launch the slam algorithm and the other required nodes use
where the arguments are set to the specific values.
`roslaunch rtabmap_drone slam.launch offb:=false control:=true vehicle:=iris_depth rtabmapviz:=true`

where vehicle can be *iris_depth*, which is the normal depth camera equipped iris from px4, which requires a modified rcS file to fuse slam position and deactivate geolocalization.

