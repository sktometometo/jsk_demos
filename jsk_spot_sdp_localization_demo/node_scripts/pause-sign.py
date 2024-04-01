#!/usr/bin/env python3

import copy
import threading
from dataclasses import dataclass
from typing import Optional

import PyKDL
import rospy
from nav_msgs.msg import Odometry
from ros_lock import ROSLock
from smart_device_protocol import UWBSDPInterface
from spot_ros_client import SpotROSClient
from uwb_localization.msg import SDPUWBDeviceArray


class PauseSignExecutor:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotROSClient()
        self._lock_mobility = ROSLock("mobility")

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices", SDPUWBDeviceArray, self._cb_device
        )

        self._sdp_interface = UWBSDPInterface()

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
        with self._lock_odom:
            if self._odom_to_base is None:
                return
        dev_ifs = copy.deepcopy(self._sdp_interface.device_interfaces)
        for dev in msg.devices:
            if dev.device_name not in [v["device_name"] for v in dev_ifs.values()]:
                continue
            frame_odom_to_device = PyKDL.Frame(
                PyKDL.Rotation(),
                PyKDL.Vector(
                    dev.point.x,
                    dev.point.y,
                    dev.point.z,
                ),
            )
