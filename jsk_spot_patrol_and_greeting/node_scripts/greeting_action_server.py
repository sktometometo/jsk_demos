#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import datetime
import math
import rospy

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Quaternion

from spot_ros_client.libspotros import SpotRosClient
from spot_ros_client.libspotros import get_nearest_person_pose
from spot_ros_client.libspotros import get_diff_for_person
from sound_play.libsoundplay import SoundClient
from jsk_robot_startup.email_topic_client import EmailTopicClient
from image_view2.image_capture_utils import ImageCaptureClient
from gdrive_ros.gdrive_ros_client import GDriveROSClient


class GreetingActionServer:

    def __init__(self):

        self.spot_ros_client = SpotRosClient()
        self.sound_client = SoundClient(
            sound_action='/robotsound_jp', sound_topic='/robotsound_jp')
        self.email_client = EmailTopicClient()
        self.image_capture_client = ImageCaptureClient()
        self.gdrive_client = GDriveROSClient()

        self.receiver_address = rospy.get_param('~receiver_address')
        self.capture_image_topic = rospy.get_param('~capture_image_topic')
        self.parents_path = rospy.get_param('~parents_path', '/spot_patrol_and_greeting/images')
        self.media_directory = rospy.get_param('~media_directory', '/tmp')

        self.srv_greeting = rospy.Service('~greeting', Trigger, self.handler)

        rospy.loginfo('initialized')

    def take_a_shot_and_send_mail(self, message):

        current = datetime.datetime.now()
        file_name = 'greeting_demo_{}_{}_{}_{}_{}_{}.jpg'.format(
            current.year,
            current.month,
            current.day,
            current.hour,
            current.minute,
            current.second)

        # Taking a shot
        file_path = self.media_directory + '/' + file_name
        ret, message = self.image_capture_client.capture(
            self.capture_image_topic, file_path)
        if not ret:
            rospy.logerr('failed to capture a image {} from {}'.format(
                file_path, self.capture_image_topic))
            return False

        # Uploading
        rospy.sleep(1)
        ret = self.gdrive_client.upload_file(
            file_path, file_name, parents_path=self.parents_path)
        if not ret[0]:
            rospy.logerr('failed to upload a image {} : {}'.format(file_path, ret[1]))
            return False
        file_url = ret[2]

        # Send a mail
        mail_title = 'Greeting Demo'
        mail_body = '{}\n{} : {}'.format(message, file_name, file_url)
        self.email_client.send_mail(
            mail_title,
            self.receiver_address,
            mail_body)

        rospy.loginfo('sent a mail')
        return True

    def head_for_person(self, use_pitch=True):

        self.spot_ros_client.pub_body_pose(0, Quaternion(x=0, y=0, z=0, w=1))
        pose = get_nearest_person_pose()
        if pose is None:
            return False
        pitch, yaw = get_diff_for_person(pose)
        rospy.loginfo('pitch:{}, yaw:{}'.format(pitch, yaw))
        self.spot_ros_client.trajectory(0, 0, yaw, 5, blocking=True)
        if use_pitch:
            self.spot_ros_client.pub_body_pose(0, Quaternion(
                x=0, y=math.sin(-pitch/2), z=0, w=math.cos(-pitch/2)))
        return True

    def stand_straight(self):

        self.spot_ros_client.pub_body_pose(0, Quaternion(x=0, y=0, z=0, w=1))

    def handler(self, req):

        greeting_message = 'こんにちは'

        ret = self.head_for_person()
        if not ret:
            res = TriggerResponse()
            res.success = False
            res.message = 'Failed to head for person'
            return res
        self.sound_client.say(greeting_message, blocking=True)
        self.stand_straight()

        success = self.take_a_shot_and_send_mail(greeting_message)
        res = TriggerResponse()
        res.success = success
        return res


if __name__ == '__main__':

    rospy.init_node('greeting_action_server')
    node = GreetingActionServer()
    rospy.spin()
