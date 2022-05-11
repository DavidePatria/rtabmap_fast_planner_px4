<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->


- [rtabmap_drone](#rtabmap_drone)
    - [Important notes:](#important-notes)
    - [Commands for mapping with realsense D435i](#commands-for-mapping-with-realsense-d435i)
    - [Dependencies](#dependencies)
        - [PX4 v1.12.3](#px4-v1123)
    - [Usage](#usage)
        - [simulations from original package](#simulations-from-original-package)
        - [modified simulations](#modified-simulations)
    - [particular configuration](#particular-configuration)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# rtabmap_drone
2D navigation example of a drone using [move_base](http://wiki.ros.org/move_base) with [mavros](http://wiki.ros.org/mavros)/[px4](https://github.com/PX4/PX4-Autopilot) and [rtabmap](wiki.ros.org/rtabmap_ros) visual SLAM. 

Overview video (click to watch on Youtube):

[![Youtube](https://i.imgur.com/UKLtD7L.gif)](https://youtu.be/A487ybS7E4E)

## Important notes:

Given the need to test this package in different scenarios and in the future employ a drone that isn't the default one used by gazebo simulations some changes have been made.

The original version of this package spawns a drone specifically configured for rtabamap, with a projection in front of it and other additions.
In particular the launch sources a px4 parameter file that is custom made to make the backed in simulation run smoothly.
By changing this and copying the usual mechanism used in many other places it ensures to launch a more generic model not aimed at performances in the specific showcase but rather at simulation fidelity.

For this reason some files have been moved modified to accomplish this task and spawn a regular black drone.

One problem is that built-in iris model is well provided with links to camera and other apparati, this would need to be included, but it might be more time sentitive to directly setup with the holibro model.


## Dependencies

Tested on ROS Melodic and ROS Noetic with the corresponding PX4 versions below.

```bash
sudo apt install \
   ros-$ROS_DISTRO-gazebo-dev \
   ros-$ROS_DISTRO-joy \
   ros-$ROS_DISTRO-imu-complementary-filter \
   ros-$ROS_DISTRO-teleop-twist-joy \
   ros-$ROS_DISTRO-geographic-msgs \
   ros-$ROS_DISTRO-dwa-local-planner \
   ros-$ROS_DISTRO-move-base \
   ros-$ROS_DISTRO-explore-lite \
   ros-$ROS_DISTRO-mavlink \
   libgeographic-dev \
   geographiclib-tools \
   libgstreamer1.0-dev

# May need this on Melodic to avoid error about silt_gazebo 
# and gstreamer (https://github.com/PX4/PX4-Autopilot/issues/13117):
sudo apt-get install libgstreamer-plugins-base1.0-dev
   
# If rtabmap is not already built from source:
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

### PX4 v1.12.3
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.12.3
git submodule update --init --recursive
pip install numpy toml packaging jinja2 empy numpy
make px4_sitl_default gazebo
# (do ctrl-c in terminal to close gazebo)
echo "source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot:~/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc
source ~/.bashrc

cd ~/catkin_ws/src
# To work with PX4/Firmware 1.12.3, mavros 1.8.0 or 1.9.0 releases should be used
# (With mavros master branch there are a lot of "Detected jump back in time" TF errors)
git clone https://github.com/mavlink/mavros.git && cd mavros && git checkout 1.8.0 && cd ..
git clone https://github.com/SyrianSpock/realsense_gazebo_plugin.git

sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

cd ~/catkin_ws
catkin_make
```

## Usage

The original package uses a custom iris model that doesn't play nice when put in other environments other than the custom one created for the repository.
In order to have simulations with a dry iris the launch files had to be distinguished and modified.
Therefore the files that contain "iris“ in their name are specific to dry iris.

Below procedures for both original and added simulations are described

### simulations from original package

```
roslaunch rtabmap_drone gazebo.launch
roslaunch rtabmap_drone slam.launch
roslaunch rtabmap_drone rviz.launch

# Arm and take off:
rosrun offboard_safety offboard_node

# Publish on control topic (meant for remote computer in real drone case). refer to the other
# Package for an explanation of the usefulness of this
rostopic pub -r 5 /remote_beat std_msgs/Empty "{}"

# Frontier exploration:
roslaunch rtabmap_drone explore.launch

# Places of interest labeling (when the drone is in the desired location):
rosservice call /rtabmap/set_label 0 Room1

# Patrolling (stopping in place for 5 seconds):
rosrun rtabmap_ros patrol.py _time:=5 Room1 Room2 Room3
```
in alternative to manually launching `offboard_node` and publishing on the topic
`roslaunch rtabmap_drone gazebo.launch offb:=true`
which will start the node and another script that publishes, useful for simulating, where safety isn't needed.


 * Manual control: If a joystick is plugged, you can send twists by holding L1 and moving the joysticks. Hold L1+L2 with left joystick down to land (be gentle to land smoothly), then hold left joystick in bottom-right position to disarm after the drone is on the ground.
 * Autonomous control: use "2D Nav Goal" button in RVIZ to set a goal to reach 

![](https://raw.githubusercontent.com/matlabbe/rtabmap_drone/master/doc/example.jpg)

### modified simulations

<!-- `rviz.launch` stays the same, but `slam_iris.launch` is used instead of the other one as the topic names are different and were changed. -->
<!---->
<!-- Spawning the drones is integrated into the launch files for gazebo, so is "iris" is specified a vanilla drone is spawned instead of the custom one. -->

The new simulation mechanism relays on the `offboard_node` which is crucial to redirect the position as computerd by rtabmap, therefore the node launch is included in `slam.launch`, where arguments are also provided to deactivate various parts of the simulation. 
Check the launch file for more details.




## particular configuration

In a commit the issue relative to the configuration of `rtabmapviz` has been resolved an a `.ini` file has been added to the repo and it is passed to `rtabmap.launch` as an argument.
But, butt, is contains a path to `$HOME` inside which it was generated, therefore it might cause problems. 
Therefore, if after launching `slam.launch` the ciclets in rtabmapviz are too big the configuration hasn't been applied.


# miscellanea

## Commands for mapping with realsense D435i

From [this wiki](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping), the commands are the following:

To launch the stream from the camera:
```
roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
     enable_accel:=true
```

To launch the imu node:
```
rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu

```

To launch rtabviz and starta mapping:
```
roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu
```

