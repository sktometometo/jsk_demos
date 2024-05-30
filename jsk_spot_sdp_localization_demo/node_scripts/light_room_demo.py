#!/usr/bin/env python3

import copy
import math
import threading
from operator import is_
from typing import Dict, List, Optional

import PyKDL
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_spot_lib.look_at_client import SpotLookAtClient
from nav_msgs.msg import Odometry
from ros_lock import ROSLock, roslock_acquire
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray


class LightRoomDemo:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sound_client = SoundClient()
        self._sdp_interface = UWBSDPInterface()

        self._sdp_interface.register_interface_callback(
            ("Light status", "?"),
            self._cb_sdp,
        )

        self._turn_on_distance_threshold: float = 3.0

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._odom_to_people: List[PyKDL.Frame] = []
        self._lock_people = threading.Lock()

        self._light_status_table_lock = threading.Lock()
        self._light_status_table: Dict[str, bool] = {}

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._sub_people_bbox = rospy.Subscriber(
            "/spot_recognition/bbox_array",
            BoundingBoxArray,
            self._cb_people_bbox,
        )

        self._running_demo_thread = True
        self._thread_demo = threading.Thread(target=self.demo)
        self._thread_demo.start()

        print("initialized")

    @property
    def odom_to_base(self):
        with self._lock_odom:
            return copy.deepcopy(self._odom_to_base)

    @property
    def odom_to_people(self):
        with self._lock_people:
            return copy.deepcopy(self._odom_to_people)

    def _cb_odom(self, msg: Odometry):
        with self._lock_odom:
            self._odom_to_base = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ),
                PyKDL.Vector(
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ),
            )

    def _cb_sdp(self, src_address, data_frame: DataFrame):
        device_content = data_frame.content[0]
        if src_address not in self._sdp_interface.device_interfaces:
            rospy.logerr(f"Failed to find device interface for {src_address}")
            return
        device_name = self._sdp_interface.device_interfaces[src_address]["device_name"]
        with self._light_status_table_lock:
            self._light_status_table[device_name] = device_content

    def _cb_people_bbox(self, msg: BoundingBoxArray):
        with self._lock_people:
            self._odom_to_people = [
                PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        msg_box.pose.orientation.x,
                        msg_box.pose.orientation.y,
                        msg_box.pose.orientation.z,
                        msg_box.pose.orientation.w,
                    ),
                    PyKDL.Vector(
                        msg_box.pose.position.x,
                        msg_box.pose.position.y,
                        msg_box.pose.position.z,
                    ),
                )
                for msg_box in msg.boxes
            ]

    def turn_light(self, device_name: str, status: str, timeout: float = 20.):
        print("send frame")
        self._sdp_interface.send(
            device_name,
            DataFrame(
                packet_description="Light control",
                content=[status],
            ),
        )
        print("sent frame")
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rospy.sleep(1.0)
            try:
                print(f"status: {self._light_status_table[device_name]}")
                if self._light_status_table[device_name] == status:
                    break
            except KeyError:
                pass

    def no_people_behind(self):
        odom_to_base = self.odom_to_base
        return (
            len(
                [
                    odom_to_base.Inverse() * person_frame
                    for person_frame in self.odom_to_people
                    if (odom_to_base.Inverse() * person_frame).p[0] < 0
                ]
            )
            == 0
        )

    def no_people_around(self):
        return len(self.odom_to_people) == 0

    def stop_demo(self):
        self._running_demo_thread = False
        self._thread_demo.join()

    def demo(self):
        self.is_out = True
        rospy.logwarn('Demo Started')
        while not rospy.is_shutdown() and self._running_demo_thread:
            rospy.sleep(1)
            with self._light_status_table_lock:
                for device_name, status in self._light_status_table.items():
                    device_interfaces = self._interface.device_interfaces
                    for addr, dev_if in device_interfaces.items():
                        if dev_if["device_name"] == device_name:
                            distance = dev_if["distance"]
                            print("==========================")
                            print(f"device_name: {device_name}")
                            print(f"distance: {distance}")
                            print(f"status: {status}")
                            print(f"is_out: {self.is_out}")
                            if distance is None:
                                print("fuga")
                                continue
                            if distance < self._turn_on_distance_threshold and self.is_out:
                                self.is_out = False
                                print("is_out -> False")
                                if not status:
                                    print("turn on")
                                    self.turn_light(device_name, True)
                            elif distance >= self._turn_on_distance_threshold and not self.is_out:
                                self.is_out = True
                                print("is_out -> True")
                                if status and self.no_people_behind():
                                    print("turn off")
                                    #self.turn_light(device_name, False)
                            else:
                                print("hoge")
                            print(f"is_out2: {self.is_out}")
        rospy.logwarn('Demo stopped')

    def walk(self):
        default_7f_walk_path = '/home/spot/default_7f.walk'
        target_id_73b1 = 'sudden-rook-a88CxeUG38pvEQyTxEyeSg=='
        target_id_73b2 = 'jawed-pigeon-9xLW4VzxDmzeP6yWPYjBzw=='
        target_id_73a4 = 'lilac-ibis-iMlKN8Hnq3se6cWyton93g==,'
        start_id = 'swept-kiwi-0phR6suSB7SB92YYWzbHKw=='
        goal_id = 'mussy-rodent-yu6WUIh8.8HhMVd+K06gFA=='

        self._client.upload_graph(default_7f_walk_path)
        self._client.set_localization_fiducial()
        self._client.navigate_to(start_id, blocking=True)

        rospy.logwarn('Start')

        # 73B1
        self._client.navigate_to(target_id_73b1, blocking=True)
        no_people_around = self.no_people_around()
        if not no_people_around:
            self._sound_client.say("Please make sure to lock all doors and windows before leaving the house")
        self._client.navigate_to(target_id_73b2, blocking=False)
        if no_people_around:
            self.turn_light("SDP Switchbot 73B1", False)
        self._client.wait_for_navigate_to_result()

        # 73B2
        no_people_around = self.no_people_around()
        if not no_people_around:
            self._sound_client.say("Please make sure to lock all doors and windows before leaving the house")
        self._client.navigate_to(target_id_73a4, blocking=False)
        if no_people_around:
            self.turn_light("SDP Switchbot", False)
        self._client.wait_for_navigate_to_result()

        # 73A4
        no_people_around = self.no_people_around()
        if not no_people_around:
            self._sound_client.say("Please make sure to lock all doors and windows before leaving the house")
        self._client.navigate_to(goal_id, blocking=False)
        if no_people_around:
            self.turn_light("SDP Switchbot 73A4", False)
        self._client.wait_for_navigate_to_result()

        rospy.logwarn('End')


if __name__ == "__main__":
    rospy.init_node("light_room_demo")
    node = LightRoomDemo()
    node.walk()
    node.stop_demo()
