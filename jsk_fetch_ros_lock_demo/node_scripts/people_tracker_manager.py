#!/usr/bin/env python

import cv2
import cv_bridge
import rospy
import message_filters

from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ObjectArray
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image

from geometry_msgs.msg import PoseStamped


class TrackedPerson(object):

    def __init__(self,
                 track_id=None,
                 image=None,
                 person_id=None,
                 pose=None
                 ):

        self.track_id = None
        self.image = None
        self.person_id = None
        self.pose = PoseStamped()


class PeopleTrackerManager(object):

    def __init__(self):

        self.cv_bridge = cv_bridge.CvBridge()
        self.tracked_people_list = {}

        mf_sub_class = message_filters.Subscriber('~input_class', classificationResult)
        mf_sub_rects = message_filters.Subscriber('~input_rects', RectsArray)
        mf_sub_image = message_filters.Subscriber('~input_image', Image)

        self.ts = message_filters.TimeSynchronizer([mf_sub_class, mf_sub_rects, mf_sub_image], 10)
        self.ts.registerCallback(self.callback)

    def callback(self,
                 msg_class,
                 msg_rects,
                 msg_image
                 ):

        


if __name__ == '__main__':
    rospy.init_node('people_tracker_manager')
    node = PeopleTrackerManager()
    rospy.spin()
