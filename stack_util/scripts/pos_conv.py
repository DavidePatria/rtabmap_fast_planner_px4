#!/usr/bin/env python3
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
import rospy


class Conversion():

    def __init__(self):
        self.pos_cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.pos_cmd_cb, queue_size=10)
        self.att_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    def pos_cmd_cb(self, msg):
        goal_att = PositionTarget()
        # according to the same programme as below it is 1 in uint
        goal_att.coordinate_frame = 1
        # velocity control mask computed from the cpp programme in offboard_safety
        goal_att.type_mask = 3064
        goal_att.position.x = msg.position.x
        goal_att.position.y = msg.position.y
        goal_att.position.z = msg.position.z
        goal_att.yaw = msg.yaw

        self.att_pub.publish(goal_att)


if __name__ == "__main__":
    rospy.init_node("pos_cmd_2_att_targ")
    conv = Conversion()
    rospy.spin()
