#!/usr/bin/env python3

import copy
import math
import threading
from typing import Dict, List, Optional

import PyKDL
import rospy
from jsk_spot_lib.look_at_client import SpotLookAtClient
from nav_msgs.msg import Odometry
from ros_lock import ROSLock, roslock_acquire
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame, UWBSDPInterface)
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray
from jsk_recognition_msgs.msg import BoundingBoxArray


class LightRoomDemo:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sdp_interface = UWBSDPInterface()

        self._sdp_interface.register_interface_callback(
            ("Light status", "?"),
            self._cb_sdp,
        )

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._odom_to_people: List[PyKDL.Frame] = []
        self._lock_people = threading.Lock()

        self._light_status_table_lock = threading.Lock()
        self._light_status_table: Dict[str, bool] = {}

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices", SDPUWBDeviceArray, self._cb_device
        )
        print("initialized")

    @property
    def odom_to_base(self):
        with self._lock_odom:
            return copy.deepcopy(self._odom_to_base)

    @property
    def people(self)
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

    def _cb_device(self, msg: SDPUWBDeviceArray):
        if len(msg.devices) == 0:
            rospy.logwarn("lengh of meessag is zero. skpped")
            return

    def _cb_people_bbox(self, msg: BoundingBoxArray):
        with self._lock_people:
            self._odom_to_people = [
                PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        msg_box.pose.pose.orientation.x,
                        msg_box.pose.pose.orientation.y,
                        msg_box.pose.pose.orientation.z,
                        msg_box.pose.pose.orientation.w,
                    ),
                    PyKDL.Vector(
                        msg_box.pose.pose.position.x,
                        msg_box.pose.pose.position.y,
                        msg_box.pose.pose.position.z,
                    ),
                )
                for msg_box in msg.boxes:
            ]

    def turn_light(self, device_name: str, status: str):
        self._sdp_interface.send(
            device_name,
            DataFrame(
                packet_description="Light control",
                content=[status],
            ),
        )
        while not rospy.is_shutdown():
            rospy.sleep(1.)
            with self._light_status_table_lock:
                try:
                    if self._light_status_table[device_name] == status:
                        break
                except KeyError:
                    pass

    def no_people_behind(self):
        odom_to_base = self.odom_to_base
        return len([odom_to_base.Inverse() * person_frame for person_frame in people if (odom_to_base.Inverse() * person_frame).p[0] < 0]) == 0

    def demo(self):

        while not rospy.is_shutdown():
            with self._light_status_table_lock:
                for device_name, status in self._light_status_table.items():
                    device_interfaces = self._interface.device_interfaces
                    for addr, dev_if in device_interfaces.items():
                        if dev_if["device_name"] == device_name:
                            distance = dev_if["distance"]
                            print(f"device_name: {device_name}")
                            print(f"distance: {distance}")
                            print(f"status: {status}")
                            if distance is None:
                                continue
                            if distance < 5.0:
                                if not status:
                                    print("turn on")
                                    self.turn_light(device_name, True)
                            else:
                                if status and self.no_people_behind():
                                    print("turn off")
                                    self.turn_light(device_name, False)

            rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("hoge")
    node = LightRoomDemo()
    node.demo()
