#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray
from std_srvs.srv import Trigger, TriggerResponse

class PersonSelect(object):

    def __init__(self):

        self.sub_image = rospy.Subscriber(rospy.resolve_name('~input'),
                                          RectArray, self.rectarray_cb, queue_size=1)
        self.pub_selected_rect = rospy.Publisher('~output_rect',RectArray,queue_size=1)

        # temporary method to detect following person
        # TODO: please detect human pose
        self.human_width_threshold = rospy.get_param('~human_width_threshold', 0.45) # w/h rate
        self.human_yaw_threshold = rospy.get_param('~human_yaw_threshold', 1.0) # rad
        self.flag_server = False
        self.server_trigger = rospy.Service('~select_person', Trigger, self.trigger_cb)

        try:
            msg_panorama_image = rospy.wait_for_message(
                '~panorama_image', Image, 10)
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('{}'.format(e))
            sys.exit(1)
        self.paranoma_width = msg_panorama_image.width
        self.paranoma_height = msg_panorama_image.height

    def rectarray_cb(self, msg):

        if not self.flag_server:
            return

        if self.paranoma_width is None or self.paranoma_height is None:
            return

        closest_person_rect = None
        max_width = 0
        for rect in msg.rects:
            yaw =  (self.paranoma_width / 2 - (rect.x + rect.width/2)) / float(self.paranoma_width) * 2 * np.pi
            if rect.width < self.human_width_threshold * rect.height and np.abs(yaw) < self.human_yaw_threshold:
                if rect.width > max_width:
                    max_width = rect.width
                    closest_person_rect = rect

        if closest_person_rect is not None:
            rospy.loginfo("find a good person to follow!")
            rect_msg = RectArray()
            rect_msg.header = msg.header
            rect_msg.rects.append(closest_person_rect)
            self.pub_selected_rect.publish(rect_msg)
            self.flag_server= False

    def trigger_cb(self, req):

        self.flag_server = not self.flag_server
        return TriggerResponse(True,'Succeeded')


if __name__ == '__main__':
    rospy.init_node('person_select')
    person_select = PersonSelect()
    rospy.spin()

