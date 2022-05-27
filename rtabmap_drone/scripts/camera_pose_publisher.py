#!/usr/bin/env python
import rospy

# Imports
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

if __name__ == '__main__':
    rospy.init_node('camera_pose_publisher')

    # In the original file the frames are passed as arguments in the launch file
    # which means they are parameters available in ros server parameters.
    # parent_frame = rospy.get_param('~parent_frame', 'map')
    # camera_frame = rospy.get_param('~child_frame','camera_link')
    # pose_topic = rospy.get_param('~pose_topic','camera/pose')

    # base_link rs200_camera


# import tf
    # listener = tf.TransformListener()
    # listener.lookupTransform('map', 'base_link', rospy.Time.now())

    # message to store the transform
    trans = TransformStamped()

    pose_pub = rospy.Publisher('camera/pose', PoseStamped, queue_size=1)
    TFbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(TFbuffer)

    rate = rospy.Rate(30.0)

    # while not TFbuffer.can_transform('map', 'camera_link_rot', rospy.Time.now(), rospy.Duration(1.0)):
    while not rospy.is_shutdown():
        try:
            # camera_link_rot is solely used to create a transform to publish the right point cloud
            trans = TFbuffer.lookup_transform('map', 'camera_link_rot', rospy.Time.now(), rospy.Duration(1.0))
        # except using tf and not tf2_ros
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
        # except:
            rate.sleep()
            continue

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = trans.transform.translation.x
        pose_msg.pose.position.y = trans.transform.translation.y
        pose_msg.pose.position.z = trans.transform.translation.z

        pose_msg.pose.orientation.x = trans.transform.rotation.x
        pose_msg.pose.orientation.y = trans.transform.rotation.y
        pose_msg.pose.orientation.z = trans.transform.rotation.z
        pose_msg.pose.orientation.w = trans.transform.rotation.w

        pose_pub.publish(pose_msg)

        rate.sleep()
