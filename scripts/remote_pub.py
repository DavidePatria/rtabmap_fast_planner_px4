#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty


class RemotePub():
    """
        pretend there's a remote computer publishing to the safety topic for now
    """

    def __init__(self):
        rospy.init_node("remote_computer_dummy")
        pub = rospy.Publisher('/remote_beat', Empty, queue_size=1)
        msg = Empty()

        rate = rospy.Rate(3)
        rospy.loginfo("starting remote beat publisher")

        while not rospy.is_shutdown():

            pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        RemotePub()
    except rospy.ROSInterruptException:
        pass
