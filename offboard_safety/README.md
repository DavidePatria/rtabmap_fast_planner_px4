# reference

This package is a spin-off (a rib) started from [rtabmap_drone_example](https://github.com/matlabbe/rtabmap_drone_example) and serves the purpose of providing fallbacks and safety mechanisms for navigating using a drone that localises through vision_pose in px4 and has to navigate autonomously.

In prticular, the implemented mechanisms are: 
- keep moving if a remote computer publishes on `/remote_beat` at a given frequency (set to 2hz now)
- stop and keep position if the above condition is not satisfied
- service to modify the status of the drone: set it to land or takeoff through a service called `custom/make_takeoff`.

## 

This package moves the drone by sending position setpoint to a mavros topic using the message `PositionTarget`.
