#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
import actionlib
import rospy
import PyKDL

from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion
from jsk_spot_delivery_demo.msg import ExecuteTaskAction
from jsk_spot_delivery_demo.msg import ExecuteTaskResult
from jsk_spot_delivery_demo.msg import PickupPackageAction
from jsk_spot_delivery_demo.msg import PickupPackageResult
from jsk_spot_delivery_demo.msg import DeliveryTask
from jsk_spot_delivery_demo.msg import DeliveryTaskArray

from spot_ros_client.libspotros import SpotRosClient
from spot_ros_client.libspotros import calc_distance_to_pose
from spot_ros_client.libspotros import get_nearest_person_pose
from spot_ros_client.libspotros import get_diff_for_person
from sound_play.libsoundplay import SoundClient
from ros_speech_recognition import SpeechRecognitionClient


class DeliveryActionServer:

    def __init__(self):

        self.srvclient_reset_body_force = rospy.ServiceProxy(
            '~reset_body_force', Empty)

        self.speech_recognition_client = SpeechRecognitionClient()
        self.spot_ros_client = SpotRosClient()
        self.sound_client = SoundClient(
            sound_action='/robotsound_jp', sound_topic='/robotsound_jp')

        self.node_list = rospy.get_param('~nodes', {})

        self.task_array = DeliveryTaskArray()
        self.pub_task_array = rospy.Publisher(
            '~task_array',
            DeliveryTaskArray,
            queue_size=1
        )

        self.actionserver_execute_task = actionlib.SimpleActionServer(
            '~execute_task',
            ExecuteTaskAction,
            self.callback_execute_task,
            auto_start=False)
        self.actionserver_pickup_package = actionlib.SimpleActionServer(
            '~pickup_package',
            PickupPackageAction,
            self.callback_pickup_package,
            auto_start=False)
        self.actionserver_execute_task.start()
        self.actionserver_pickup_package.start()

        rospy.loginfo('initialized')

    def spin(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publish_task_array()

    def publish_task_array(self):

        self.task_array.header.stamp = rospy.Time.now()
        self.pub_task_array.publish(self.task_array)

    def approach_person(self, distance_to_person=1.0):

        self.stand_straight()
        pose = get_nearest_person_pose()
        if pose is None:
            return False
        if calc_distance_to_pose(pose) < 2.0:
            return True
        else:
            pos = PyKDL.Vector(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z)
            theta = math.atan2(pos[1], pos[0])
            pos = pos - distance_to_person * pos / pos.Norm()
            x = pos[0]
            y = pos[1]
            self.spot_ros_client.trajectory(
                x,
                y,
                theta,
                2,
                blocking=True)
        return True

    def head_for_person(self, use_pitch=True, yaw_offset=0):

        self.stand_straight()
        pose = get_nearest_person_pose()
        if pose is None:
            return False
        pitch, yaw = get_diff_for_person(pose)
        self.spot_ros_client.trajectory(
            0,
            0,
            yaw + yaw_offset,
            5,
            blocking=True)
        if use_pitch:
            self.spot_ros_client.pubBodyPose(0, Quaternion(
                x=0, y=math.sin(-pitch/2), z=0, w=math.cos(-pitch/2)))
            self.spot_ros_client.stand()
        return True

    def stand_straight(self):

        self.spot_ros_client.pubBodyPose(0, Quaternion(x=0, y=0, z=0, w=1))

    def wait_package_setting(self, duration=rospy.Duration(120)):

        self.done_pick_or_place = False
        force_threshold = rospy.get_param('~force_threshold', 5)
        self.srvclient_reset_body_force()

        def callback(msg):
            rospy.logdebug('force z: {}, threshold: {}'.format(
                math.fabs(msg.wrench.force.z), force_threshold))
            if math.fabs(msg.wrench.force.z) > force_threshold:
                self.done_pick_or_place = True
        sub = rospy.Subscriber('~body_force', WrenchStamped, callback)
        rate = rospy.Rate(10)
        timeout_deadline = rospy.Time.now() + duration
        while not rospy.is_shutdown():
            rate.sleep()
            if self.done_pick_or_place or rospy.Time.now() > timeout_deadline:
                break
        success = self.done_pick_or_place
        sub.unregister()
        del self.done_pick_or_place
        return success

    def check_allow_word(self, text):

        self.allow_words = [
            'はい',
            'あります',
            'お願い',
            'イエス',
        ]
        for word in self.allow_words:
            if word in text:
                return True
        return False

    def callback_pickup_package(self, goal):

        success, message = self.pickup_package(goal.timeout)

        if success:
            if goal.execute_after_pickup:
                success, message =\
                        self.execute_task(len(self.task_array.tasks)-1)
                result = PickupPackageResult()
                result.success = success
                result.message = message
                if success:
                    self.actionserver_pickup_package.set_succeeded(result)
                else:
                    self.actionserver_pickup_package.set_succeeded(result)
            else:
                result = PickupPackageResult()
                result.success = success
                result.message = message
                self.actionserver_pickup_package.set_succeeded(result)
        else:
            result = PickupPackageResult()
            result.success = success
            result.message = message
            self.actionserver_pickup_package.set_aborted(result)

    def pickup_package(self, timeout=rospy.Duration(120)):

        timeout_deadline = rospy.Time.now() + timeout
        rate = rospy.Rate(1)

        rospy.loginfo('Asking package task')
        success = False
        timeout_temp = rospy.Time.now() + rospy.Duration(20)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_temp:
            if not self.approach_person() or not self.head_for_person():
                rospy.logerr('No Person')
                rate.sleep()
                continue
            self.sound_client.say('配達物はありませんか', blocking=True)
            recognition_result = self.speech_recognition_client.recognize()
            if len(recognition_result.transcript) == 0:
                rospy.logerr(
                    'No matching node found from spoken \'{}\''.format(
                        recognition_result))
                self.sound_client.say('聞き取れませんでした', blocking=True)
                continue
            else:
                recognized_text = recognition_result.transcript[0]
                rospy.loginfo('recognized_text: {}'.format(recognized_text))
                success = True
                break

        if not success or not self.check_allow_word(recognized_text):
            self.stand_straight()
            return False, 'No delivery task'

        rospy.loginfo('Asking package information')
        success = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_deadline:
            if not self.head_for_person():
                rospy.logerr('No Person')
                rate.sleep()
                continue
            self.sound_client.say('配達先を教えてください。', blocking=True)
            recognition_result = self.speech_recognition_client.recognize()
            if len(recognition_result.transcript) == 0:
                rospy.logerr(
                    'No matching node found from spoken \'{}\''.format(
                        recognition_result))
                self.sound_client.say('配達先がわかりませんでした', blocking=True)
                continue
            recognized_destination = recognition_result.transcript[0]
            target_node_candidates = {}
            for node_id, value in self.node_list.items():
                try:
                    if 'name_jp' not in value:
                        continue
                    if type(value['name_jp']) is list:
                        # DO HOGE
                        for name in value['name_jp']:
                            if name.encode('utf-8') == recognized_destination:
                                target_node_candidates[node_id] = value
                    else:
                        if value['name_jp'].encode('utf-8') ==\
                                recognized_destination:
                            target_node_candidates[node_id] = value
                except Exception as e:
                    rospy.logerr('Error: {}'.format(e))
            if len(target_node_candidates) == 0:
                rospy.logerr(
                    'No matching node found from spoken \'{}\''.format(
                        recognition_result))
                self.sound_client.say('配達先がわかりませんでした', blocking=True)
            else:
                success = True
                break

        if not success:
            self.stand_straight()
            return False, 'Falied to recognize the destination from speech.'
        else:
            target_node_id = target_node_candidates.keys()[0]
            if isinstance(self.node_list[target_node_id]['name_jp'], list):
                target_node_name_jp =\
                    self.node_list[target_node_id]['name_jp'][0].encode(
                        'utf-8')
            else:
                target_node_name_jp =\
                    self.node_list[target_node_id]['name_jp'].encode(
                        'utf-8')
            rospy.loginfo('target_node_id: {}'.format(target_node_id))
            self.sound_client.say('{}ですね。わかりました。'.format(
                target_node_name_jp), blocking=True)

        rospy.loginfo('Asking sender information')
        success = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_deadline:
            if not self.head_for_person():
                rospy.logerr('No Person')
                rate.sleep()
                continue
            self.sound_client.say('送り主の名前を教えてください', blocking=True)
            recognition_result = self.speech_recognition_client.recognize()
            if len(recognition_result.transcript) == 0:
                rospy.logerr('No spoken result: \'{}\''.format(
                    recognition_result))
                self.sound_client.say('聞き取れませんでした', blocking=True)
                continue
            else:
                recognized_name = recognition_result.transcript[0]
                success = True
                break

        if not success:
            self.stand_straight()
            return False, 'Falied to recognize sender name from speech.'
        else:
            sender_name = recognized_name
            rospy.loginfo('sender name is {}'.format(sender_name))
            self.sound_client.say('{}さんですね'.format(sender_name), blocking=True)

        rospy.loginfo('Waiting for package placed.')
        self.head_for_person(use_pitch=False, yaw_offset=1.57)
        self.sound_client.say('荷物を置いてください', blocking=True)
        success = self.wait_package_setting(
            timeout_deadline - rospy.Time.now())
        if not success:
            rospy.logerr('Timeout for package placement')
            return False, 'Timeout for package placement.'
        else:
            rospy.loginfo('Package placed')
            self.sound_client.say('荷物を確認しました', blocking=True)

        task = DeliveryTask()
        task.target_node_id = target_node_id
        task.package_content = ''
        task.sender = sender_name
        self.task_array.tasks.append(task)

        return True, 'Success'

    def callback_execute_task(self, goal):

        success, message = self.execute_task(goal.index)

        if success:
            rospy.loginfo(message)
            self.actionserver_execute_task.set_succeeded(
                ExecuteTaskResult(success, message))
        else:
            rospy.logerr(message)
            self.actionserver_execute_task.set_aborted(
                ExecuteTaskResult(success, message))

    def execute_task(self, index):

        if index >= len(self.task_array.tasks):
            return False, 'goal index ({}) is out of range.'.format(index)
        task = self.task_array.tasks[index]

        rospy.loginfo('move to {}'.format(task.target_node_id))
        result = self.spot_ros_client.execute_behaviors(task.target_node_id)
        rospy.logwarn('result: {}'.format(result))
        if not result.success:
            return False, 'Failed to reach {}'.format(task.target_node_id)
        rospy.loginfo('reached {}'.format(task.target_node_id))

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo('Waiting for packag picked.')
            if not self.approach_person():
                rate.sleep()
                continue
            if not self.head_for_person(use_pitch=False, yaw_offset=1.5):
                rate.sleep()
                continue
            self.sound_client.say(
                '{}さんから{}のお届けです、荷物を受け取ってください'.format(
                    task.sender, task.package_content),
                blocking=True)
            if self.wait_package_setting(rospy.Duration(5)):
                rospy.loginfo('Package picked')
                self.sound_client.say('荷物の受取を確認しました')
                break

        return True, 'Success'


def main():

    rospy.init_node('delivery_action_server')
    node = DeliveryActionServer()
    node.spin()


if __name__ == '__main__':
    main()
