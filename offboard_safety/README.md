# reference

This package is a spin-off (a rib) started from rtabmap_drone_example and serves the purpose of providing fallbacks and safety mechanisms for navigating using a drone that localises though vision_pose in px4 and has to navigate autonomously.

In prticular, the implemented mechanisms are: 
- keep moving if a remote computer publishes on `/remote_beat` at a given frequency
- stop and keep position if the above condition is not satisfied
- service to modify the status of the drone: set it to land or takeoff

## 

This package moves the drone by sending position setpoint through a mavros topic using the message `PositionTarget`.
