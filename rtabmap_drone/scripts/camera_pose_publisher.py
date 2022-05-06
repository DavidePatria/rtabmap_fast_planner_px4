#!/usr/bin/env python
import rospy

# Imports
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('camera_pose_publisher')

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
            trans,rot = listener.lookupTransform('base_link', 'rs200_camera', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
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
