#!/usr/bin/env python
import rospy

# Imports
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('camera_pose_publisher')

    # In the original file the frames are passed as arguments in the launch file
    # which means they are parameters available in ros server parameters.
    # parent_frame = rospy.get_param('~parent_frame', 'map')
    # camera_frame = rospy.get_param('~child_frame','camera_link')
    # pose_topic = rospy.get_param('~pose_topic','camera/pose')

    # base_link rs200_camera

    pose_pub = rospy.Publisher('camera/pose', PoseStamped, queue_size=1)
    listener = tf.TransformListener()

    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        try:
            # (trans,rot) = listener.lookupTransform('/'+parent_frame, '/'+camera_frame, rospy.Time(0))
            # camera_link_rot is solely used to create a transform to publish the right point cloud
            listener.waitForTransform('map', 'camera_link_rot', rospy.Time.now(), rospy.Duration(0.2))
            trans,rot = listener.lookupTransform('map', 'camera_link_rot', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            rospy.logwarn(e)
            continue

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        pose_pub.publish(pose_msg)

        rate.sleep()
