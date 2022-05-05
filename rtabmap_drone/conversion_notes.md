<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->


- [changes](#changes)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# changes 

The executable `rgbd_sync` should be used to avoid syncing problem, the only issue being that it containes hard coded topic names.
A launch file is needed to remap the needed topics.

`rgbd_sync` subscribes to 

```
	/rgb/image
	/depth/image
	/rgb/camera_info
```

The node launched through `slam.launch` subscribes to
```
/rtabmap/rtabmap subscribed to (approx sync):
   /rtabmap/odom \
   /r200/camera/color/image_raw \
   /r200/camera/depth/image_raw \
   /r200/camera/color/camera_info \
   /rtabmap/odom_info
```

The idea is to remap twice the topics, once in input inside `rgbd_sync` so it gets the right topics, and the remap those getting into slam.launch
