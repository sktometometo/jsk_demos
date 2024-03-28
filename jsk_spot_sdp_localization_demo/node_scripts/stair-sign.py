#!/usr/bin/env python3

from ast import Dict
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
import PyKDL
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ros_lock import ROSLock
from smart_device_protocol import UWBSDPInterface
from spot_ros_client import SpotROSClient
from uwb_localization.msg import SDPUWBDeviceArray

from jsk_spot_sdp_localization_demo.msg import Stair


class StairPointType(Enum):
    TOP_A = "TOP_A"
    TOP_B = "TOP_B"
    BOTTOM_A = "BOTTOM_A"
    BOTTOM_B = "BOTTOM_B"


@dataclass
class StairEntry:
    stair_name: str = ""
    stair_top_point_a: Optional[PyKDL.Vector] = None
    stair_top_point_b: Optional[PyKDL.Vector] = None
    stair_bottom_point_a: Optional[PyKDL.Vector] = None
    stair_bottom_point_b: Optional[PyKDL.Vector] = None


def parse_device_text(
    device_name: str, device_content: str, device_point: List[float]
) -> Tuple[str, StairPointType, PyKDL.Vector]:
    device_content = device_content.strip()
    contents = device_content.split(",")
    stair_name = contents[0]
    point_type = StairPointType(contents[1])
    point = PyKDL.Vector(device_point[0], device_point[1], device_point[2])
    return stair_name, point_type, point


class StairExecutor:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotROSClient()
        self._lock_mobility = ROSLock("mobility")

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._odom_to_base: Optional[PyKDL.Frame] = None

        self._stair_tables: Dict[str, StairEntry] = {}

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
        for msg_device in msg.devices:
            stair_name, point_type, point = parse_device_text(
                msg.device_name,
                msg.device_content,
                [msg_device.point.x, msg_device.point.y, msg_device.point.z],
            )
            if stair_name not in self._stair_tables.keys():
                self._stair_tables[stair_name] = StairEntry(stair_name=stair_name)
            if point_type == StairPointType.TOP_A:
                self._stair_tables[stair_name].stair_top_point_a = point
            elif point_type == StairPointType.TOP_B:
                self._stair_tables[stair_name].stair_top_point_b = point
            elif point_type == StairPointType.BOTTOM_A:
                self._stair_tables[stair_name].stair_bottom_point_a = point
            elif point_type == StairPointType.BOTTOM_B:
                self._stair_tables[stair_name].stair_bottom_point_b = point


if __name__ == "__main__":
    rospy.init_node("stair_executor")
    executor = StairExecutor()
    rospy.spin()
