#!/usr/bin/env python3

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

        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices", SDPUWBDeviceArray, self._cb_device
        )

    def _cb_odom(self, msg: Odometry):
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
        if self._odom_to_base is None:
            return
        frame_odom_to_device = PyKDl.Frame(
            PyKDL.Rotation(),
            PyKDL.Vector(
                msg.point.x,
                msg.point.y,
                msg.point.z,
            ),
        )
