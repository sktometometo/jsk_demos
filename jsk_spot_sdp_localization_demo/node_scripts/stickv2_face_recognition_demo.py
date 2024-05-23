#!/usr/bin/env python3

import copy
import math
import threading
from operator import is_
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass

import PyKDL
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_spot_lib.look_at_client import SpotLookAtClient
from nav_msgs.msg import Odometry
from ros_lock import ROSLock, roslock_acquire
from spot_msgs.msg import GraphNavLocalization
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray


@dataclass
class TableEntry:
    device_name: str
    address = None
    nearest_node_id: Optional[str] = None
    nearest_distance: Optional[float] = None


class LightRoomDemo:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sdp_interface = UWBSDPInterface()

        self._detected_face_id = ("Detected face", "s")

        self._target_person_name = 'shinjo'
        self._trigger = False
        self._triggered_device_addr = None

        self._current_waypoint = ""

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._device_table_lock = threading.Lock()
        self._device_table: Dict[Tuple, TableEntry] = {}

        self._sub_graph_nav_localization = rospy.Subscriber(
            '/spot/graph_nav_localization_state',
            GraphNavLocalization,
            self._cb_graph_nav_localization,
        )
        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)

        print("initialized")

    @property
    def odom_to_base(self):
        with self._lock_odom:
            return copy.deepcopy(self._odom_to_base)

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
        person_name = data_frame.content[0]
        if person_name == self._target_person_name:
            self._triggered_device_addr = src_address
            self._trigger = True

    def _cb_people_bbox(self, msg: BoundingBoxArray):
        pass

    def _cb_graph_nav_localization(self, msg: GraphNavLocalization):
        self._current_waypoint = msg.waypoint_id

    def stop_demo(self):
        self._running_demo_thread = False
        self._thread_demo.join()

    def demo(self):
        default_7f_walk_path = '/home/spot/default_7f.walk'
        patrol_id = 'leafed-gnu-ZMct0zR8OTzXyQbC0RBKpw=='
        dock_id = 'fanned-craw-1uaBLxMUEpHL+J6M3F+F1w=='

        input("Start from 73B2")
        self._client.upload_graph(default_7f_walk_path)
        self._client.set_localization_fiducial()

        # Walk and find a nearest node to each camera
        self._client.navigate_to(patrol_id, blocking=False)
        rospy.logwarn('Start')
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._client.wait_for_navigate_to_result(duration=rospy.Duration(1.0)):
                break
            rate.sleep()
            dis = self._interface.device_interfaces
            current_node = self._current_waypoint
            print(dis)
            with self._device_table_lock:
                for address, entry in dis.items():
                    device_name = entry['device_name']
                    distance = entry['distance']
                    if address not in self._device_table.keys():
                        self._device_table[address] = TableEntry(address=address, device_name=device_name)
                    if distance is not None:
                        if self._device_table[address].nearest_distance is None or distance < self._device_table[address].nearest_distance:
                            self._device_table[address].nearest_distance = distance
                            self._device_table[address].nearest_node_id = current_node

        print("result: {}".format(self._device_table))
        self._client.navigate_to(dock_id, blocking=True)

        self._client.dock(521)
        self._sdp_interface.register_interface_callback(
            self._detected_face_id,
            self._cb_sdp,
        )
        while not rospy.is_shutdown() and not self._trigger:
            rate.sleep()
        entry = self._device_table[self._triggered_device_addr]
        self.client.undock()
        self.client.navigate_to(entry.nearest_node_id, blocking=True)
        rospy.logwarn('End')


if __name__ == "__main__":
    rospy.init_node("light_room_demo")
    node = LightRoomDemo()
    node.demo()
