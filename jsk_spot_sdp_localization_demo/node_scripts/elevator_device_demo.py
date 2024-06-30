#!/usr/bin/env python3


import copy
import math
import threading
from typing import Dict, List, Optional

import PyKDL
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_spot_lib.look_at_client import SpotLookAtClient
from nav_msgs.msg import Odometry
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray


class ElevatorDeviceDemo:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sdp_interface = UWBSDPInterface()

        self._sdp_interface.register_interface_callback(
            ("Elevator status", "?"),
            self._cb_sdp,
        )

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._elevator_status_table_lock = threading.Lock()
        self._elevator_status_table: Dict[str, bool] = {}

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices", SDPUWBDeviceArray, self._cb_device
        )
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

    def _cb_device(self, msg: SDPUWBDeviceArray):
        with self._elevator_status_table_lock:
            for device in msg.devices:
                self._elevator_status_table[device.device_id] = device.status

    def _cb_sdp(self, data: DataFrame):
        with self._elevator_status_table_lock:
            self._elevator_status_table[data.content[0]] = data.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update()
            rate.sleep()

    def call_elevator(self, direction: str = "up"):
        device_interfaces = self._interface.device_interfaces
        elevator_outside_devices = [
            dev_if
            for addr, dev_if in device_interfaces.items()
            if "EV_PNL_OUT" in dev_if["device_name"]
        ]
        if not elevator_outside_devices:
            rospy.logerr("No elevator outside devices found")
            return
        elevator_outside_device = sorted(
            elevator_outside_devices,
            key=lambda x: x["distance"] if x["distance"] else 1000000,
        )[0]
        self._interface.send(
            elevator_outside_device["device_name"],
            DataFrame(
                packet_description="Elevator call",
                contents=[direction],
            ),
        )

    def _update(self):
        with self._elevator_status_table_lock:
            for device_name, status in self._elevator_status_table.items():
                if status:
                    rospy.loginfo(f"Elevator {device_name} is {status}")
                else:
                    rospy.loginfo(f"Elevator {device_name} is not available")
