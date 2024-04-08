#!/usr/bin/env python3

import threading
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
from visualization_msgs.msg import Marker, MarkerArray

from jsk_spot_sdp_localization_demo.msg import Stair


class StairPointType(Enum):
    TOP_A = "TOP_A"
    TOP_B = "TOP_B"
    BOTTOM_A = "BOTTOM_A"
    BOTTOM_B = "BOTTOM_B"


@dataclass
class StairEntry:
    stair_name: str = ""

    stair_top_a_device_name: Optional[str] = None
    stair_top_point_a: Optional[PyKDL.Vector] = None

    stair_top_b_device_name: Optional[str] = None
    stair_top_point_b: Optional[PyKDL.Vector] = None

    stair_bottom_a_device_name: Optional[str] = None
    stair_bottom_point_a: Optional[PyKDL.Vector] = None

    stair_bottom_b_device_name: Optional[str] = None
    stair_bottom_point_b: Optional[PyKDL.Vector] = None


def parse_device_text(device_content: str) -> Optional[Tuple[str, StairPointType]]:
    try:
        device_content = device_content.strip()
        contents = device_content.split(",")
        stair_name = contents[0]
        point_type = StairPointType(contents[1])
        return stair_name, point_type
    except Exception as e:
        rospy.logerr(f"Failed to parse device content: {device_content}")
        return None


class StairExecutor:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotROSClient()
        self._lock_mobility = ROSLock("mobility")

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._odom_frame_id: Optional[str] = None
        self._lock_odom = threading.Lock()

        self._stair_tables: Dict[str, StairEntry] = {}
        self._lock_stair_tables = threading.Lock()

        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices", SDPUWBDeviceArray, self._cb_device
        )
        self._sdp_device_parent_frame_id: Optional[str] = None

        self._sdp_interface = UWBSDPInterface()
        self._sdp_interface.register_interface_callback(
            ("Landmark information", "S"),
            self._cb_sdp_landmark_information,
        )

        self._pub_markers = rospy.Publisher("/stair_markers", MarkerArray, queue_size=1)

    def _cb_sdp_landmark_information(self, src_address, contents: List):
        device_content = contents[0]
        ret = parse_device_text(device_content)
        if ret is None:
            return
        stair_name = ret[0]
        point_type = ret[1]
        if src_address not in self._sdp_interface.device_interfaces:
            rospy.logerr(f"Failed to find device interface for {src_address}")
            return
        device_name = self._sdp_interface.device_interfaces[src_address]["device_name"]
        with self._lock_stair_tables:
            if stair_name not in self._stair_tables:
                self._stair_tables[stair_name] = StairEntry(stair_name=stair_name)
            if point_type == StairPointType.TOP_A:
                self._stair_tables[stair_name].stair_top_a_device_name = device_name
            elif point_type == StairPointType.TOP_B:
                self._stair_tables[stair_name].stair_top_b_device_name = device_name
            elif point_type == StairPointType.BOTTOM_A:
                self._stair_tables[stair_name].stair_bottom_a_device_name = device_name
            elif point_type == StairPointType.BOTTOM_B:
                self._stair_tables[stair_name].stair_bottom_b_device_name = device_name

    def _cb_odom(self, msg: Odometry):
        with self._lock_odom:
            self._odom_frame_id = msg.header.frame_id
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
        with self._lock_stair_tables:
            for msg_device in msg.devices:
                if self._sdp_device_parent_frame_id is None:
                    self._sdp_device_parent_frame_id = msg_device.header.frame_id
                device_name = msg.device_name
                for stair_entry in self._stair_tables.values():
                    if stair_entry.stair_top_a_device_name == device_name:
                        stair_entry.stair_top_point_a = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )
                    elif stair_entry.stair_top_b_device_name == device_name:
                        stair_entry.stair_top_point_b = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )
                    elif stair_entry.stair_bottom_a_device_name == device_name:
                        stair_entry.stair_bottom_point_a = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )
                    elif stair_entry.stair_bottom_b_device_name == device_name:
                        stair_entry.stair_bottom_point_b = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )

    def publish_markers(self):
        with self._lock_stair_tables:
            msg = MarkerArray()
            for stair_name, entry in self._stair_tables.items():
                if (
                    entry.stair_top_point_a is not None
                    and entry.stair_top_point_b is not None
                    and entry.stair_bottom_point_a is not None
                    and entry.stair_bottom_point_b is not None
                ):
                    for point_name, point_type, point in [
                        ("top_a", StairPointType.TOP_A, entry.stair_top_point_a),
                        ("top_b", StairPointType.TOP_B, entry.stair_top_point_b),
                        (
                            "bottom_a",
                            StairPointType.BOTTOM_A,
                            entry.stair_bottom_point_a,
                        ),
                        (
                            "bottom_b",
                            StairPointType.BOTTOM_B,
                            entry.stair_bottom_point_b,
                        ),
                    ]:
                        marker = Marker()
                        marker.header.frame_id = self._sdp_device_parent_frame_id
                        marker.header.stamp = rospy.Time.now()
                        marker.ns = "stair"
                        marker.id = hash(stair_name + point_name + "shere")
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = point.x()
                        marker.pose.position.y = point.y()
                        marker.pose.position.z = point.z()
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 1.0
                        marker.scale.y = 1.0
                        marker.scale.z = 1.0
                        marker.color.r = 1.0
                        marker.color.a = 1.0
                        msg.markers.append(marker)

                    marker = Marker()
                    marker.header.frame_id = self._sdp_device_parent_frame_id
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "stair"
                    marker.id = hash(stair_name)
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 1.0
                    marker.color.g = 1.0
                    marker.color.a = 1.0
                    for point_name, point_type, point in [
                        ("top_a", StairPointType.TOP_A, entry.stair_top_point_a),
                        ("top_b", StairPointType.TOP_B, entry.stair_top_point_b),
                        (
                            "bottom_a",
                            StairPointType.BOTTOM_A,
                            entry.stair_bottom_point_a,
                        ),
                        (
                            "bottom_b",
                            StairPointType.BOTTOM_B,
                            entry.stair_bottom_point_b,
                        ),
                    ]:
                        if point is None:
                            continue
                        marker.points.append(
                            Point(x=point.x(), y=point.y(), z=point.z())
                        )
                    msg.markers.append(marker)

            self._pub_markers.publish(msg)

    def spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_markers()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("stair_executor")
    executor = StairExecutor()
    rospy.spin()
