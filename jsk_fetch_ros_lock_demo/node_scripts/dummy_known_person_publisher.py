#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
import tf2_ros
import math


def near_table(box):
    if math.sqrt((box.pose.position.x - 3.921)**2 + (box.pose.position.y - 7.847)**2) < 3.0:
        return True
    else:
        return False


class DummyKnownPersonPublisher(object):

    def __init__(self):

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_known = rospy.Publisher('~output_known', BoundingBoxArray, queue_size=1)
        self.pub_unknown = rospy.Publisher('~output_unknown', BoundingBoxArray, queue_size=1)

        self.sub = rospy.Subscriber('~input', BoundingBoxArray, self.callback)

    def callback(self, msg):

        msg_known = BoundingBoxArray()
        msg_unknown = BoundingBoxArray()

        msg_known.header = msg.header
        msg_unknown.header = msg.header

        for box in msg.boxes:
            if near_table(box):
                msg_known.boxes.append(box)
            else:
                msg_unknown.boxes.append(box)

        self.pub_known.publish(msg_known)
        self.pub_unknown.publish(msg_unknown)


if __name__ == '__main__':

    rospy.init_node('dummy_known_person_publisher')
    node = DummyKnownPersonPublisher()
    rospy.spin()
