#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox


class Node:

    def __init__(self):

        self.offset = rospy.get_param('~z_offset', 0.0)

        self.pub = rospy.Publisher('~output', BoundingBox, queue_size=1)
        self.sub = rospy.Subscriber('~input', BoundingBox, self.cb)

    def cb(self, msg):

        msg.pose.position.z += self.offset
        self.pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('target_bbox_offset')
    node = Node()
    rospy.spin()
