#!/usr/bin/env python

import rospy
import message_filters
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry
from ros_lock import ROSLock
from ros_lock import roslock_acquire
import threading
import math
from spot_ros_client.libspotros import SpotRosClient


class AppoachPerson(object):

    def __init__(self):

        self.timeout_acquire = rospy.get_param('~timeout_acquire', 5)
        self.index_person = int(rospy.get_param('~index_person', 0))
        self.duration_interaction = rospy.get_param(
            '~duration_interaction', 30)
        self.ros_lock = ROSLock('locomotion')

        self.lock_for_tracked_objects = threading.Lock()
        self.tracked_objects = {}

        sub_mot_bbox = message_filters.Subscriber(
            '/spot_recognition/multi_object_tracker/bbox_array', BoundingBoxArray)
        sub_mot_obj = message_filters.Subscriber(
            '/spot_recognition/multi_object_tracker/obj_array', ObjectArray)
        sub_odom = message_filters.Subscriber('/spot/odometry', Odometry)
        self.ts = message_filters.TimeSynchronizer(
            [sub_mot_bbox, sub_mot_obj, sub_odom], 10)
        self.ts.registerCallback(self.callback)

    def callback(self, msg_bbox, msg_obj, msg_odom):

        tracked_indices = [box.label for box in msg_bbox.boxes]
        with self.lock_for_tracked_objects:
            for track_id in self.tracked_objects:
                if track_id not in tracked_indices:
                    self.tracked_objects.pop(track_id)
            for box, obj in zip(msg_bbox.boxes, msg_obj.objects):
                if obj.class_id == self.index_person:
                    distance = math.sqrt(
                        (box.pose.position.x - msg_odom.pose.pose.position.x)**2 +
                        (box.pose.position.y - msg_odom.pose.pose.position.y)**2 +
                        (box.pose.position.z - msg_odom.pose.pose.position.z)
                    )
                    self.tracked_objects[box.label] = (box, obj, distance)

    def spin(self):

        spot_client = SpotRosClient()
        while not rospy.is_shutdown():
            with roslock_acquire(self.ros_lock, self.timeout_acquire):
                with self.lock_for_tracked_objects:
                    candidates = self.tracked_objects.values()
                if len(candidates) == 0:
                    continue
                candidates = sorted(candidates, lambda x: x[2])
                target_id = candidates[0]
                end_time = rospy.Time.now() + rospy.Duration(self.duration_interaction)
                rate = rospy.Rate(10)
                while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                    rate.sleep()
                    with self.lock_for_tracked_objects:
                        if target_id not in self.tracked_objects:
                            rospy.logwarn('target {} lost.'.format(target_id))
                            break
                        target = self.tracked_objects[target_id]
                    target_distance = target[2]
                    target_x = target[0].pose.position.x
                    target_y = target[0].pose.position.y
                    target_theta = math.atan2(target_y, target_x)
                    target_x = target_x * \
                        (target_distance - 1) / target_distance
                    target_y = target_y * \
                        (target_distance - 1) / target_distance
                    spot_client.trajectory(target_x, target_y, target_theta)


def main():

    rospy.init_node('approach_person')
