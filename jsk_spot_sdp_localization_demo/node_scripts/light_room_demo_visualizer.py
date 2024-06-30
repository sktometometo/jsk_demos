#!/usr/bin/env python

import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String
from uwb_localization.msg import SDPUWBDeviceArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class DeviceInfo:
    device_name: str
    header: Header = Header()
    position: Point = Point()
    distance: Optional[float] = None
    is_out: Optional[bool] = None
    status: Optional[bool] = None

    def is_complete(self) -> bool:
        return all(
            [
                self.distance is not None and self.distance <= 5.0,
                self.is_out is not None,
                self.status is not None,
                self.header.frame_id != "",
            ]
        )


class LightRoomDeviceVisualizer:

    def __init__(self):

        self._pub_marker = rospy.Publisher(
            "/light_room_device_marker_array",
            MarkerArray,
            queue_size=1,
        )

        self.device_dict: Dict[str, DeviceInfo] = dict()
        self.lock_device_dict = threading.Lock()

        self._sub_debug_string = rospy.Subscriber(
            "/light_room_demo/debug",
            String,
            self._callback_debug,
        )

        self._sub_uwb_device = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._callback_uwb_device,
        )

    def _publish_marker(self):

        marker_array = MarkerArray()
        with self.lock_device_dict:
            for device_name, device_info in self.device_dict.items():
                if not device_info.is_complete():
                    show = False
                else:
                    show = True
                #
                marker = Marker()
                marker.header = device_info.header
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD if show else Marker.DELETE
                marker.pose.position.x = device_info.position.x
                marker.pose.position.y = device_info.position.y
                marker.pose.position.z = 0.75
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 1.5
                marker.color.a = 1.0
                if device_info.is_out:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                marker_array.markers.append(marker)
                #
                marker = Marker()
                marker.header = device_info.header
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364 + 1
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD if show else Marker.DELETE
                marker.pose.position.x = device_info.position.x
                marker.pose.position.y = device_info.position.y
                marker.pose.position.z = 1.5
                marker.pose.orientation.y = 1.0 / math.sqrt(2)
                marker.pose.orientation.w = 1.0 / math.sqrt(2)
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.1
                marker.color.a = 1.0
                if device_info.is_out:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                marker_array.markers.append(marker)
                #
                marker = Marker()
                marker.header = device_info.header
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364 + 2
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD if show else Marker.DELETE
                marker.pose.position.x = device_info.position.x
                marker.pose.position.y = device_info.position.y
                marker.pose.position.z = 1.5
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.color.a = 1.0
                if device_info.status:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                for i in range(10):
                    point_outer = Point(
                        x=0.0,
                        y=1.0 * math.cos(i * 2 * math.pi / 10),
                        z=1.0 * math.sin(i * 2 * math.pi / 10),
                    )
                    point_inner = Point(
                        x=0.0,
                        y=0.7 * math.cos(i * 2 * math.pi / 10),
                        z=0.7 * math.sin(i * 2 * math.pi / 10),
                    )
                    marker.points.append(point_outer)
                    marker.points.append(point_inner)
                marker_array.markers.append(marker)
                #
                marker = Marker()
                marker.header = device_info.header
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364 + 3
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD if show else Marker.DELETE
                marker.pose.position.x = device_info.position.x
                marker.pose.position.y = device_info.position.y
                marker.pose.position.z = 2.0
                marker.pose.orientation.w = 1.0
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.scale.z = 0.5
                marker.text = device_name
                marker_array.markers.append(marker)
        self._pub_marker.publish(marker_array)

    def _callback_debug(self, msg: String):
        debug_info = yaml.safe_load(msg.data)
        if (
            "device_name" not in debug_info
            or "distance" not in debug_info
            or "is_out" not in debug_info
            or "status" not in debug_info
        ):
            rospy.logwarn("Invalid debug info: %s", debug_info)
            return
        with self.lock_device_dict:
            if debug_info["device_name"] not in self.device_dict:
                self.device_dict[debug_info["device_name"]] = DeviceInfo(
                    device_name=debug_info["device_name"],
                    distance=debug_info["distance"],
                    is_out=debug_info["is_out"],
                    status=debug_info["status"],
                )
            else:
                self.device_dict[debug_info["device_name"]].distance = debug_info[
                    "distance"
                ]
                self.device_dict[debug_info["device_name"]].is_out = debug_info[
                    "is_out"
                ]
                self.device_dict[debug_info["device_name"]].status = debug_info[
                    "status"
                ]

    def _callback_uwb_device(self, msg: SDPUWBDeviceArray):

        for uwb_dev in msg.devices:
            with self.lock_device_dict:
                if uwb_dev.device_name not in self.device_dict:
                    self.device_dict[uwb_dev.device_name] = DeviceInfo(
                        header=uwb_dev.header,
                        position=uwb_dev.point,
                        device_name=uwb_dev.device_name,
                    )
                else:
                    self.device_dict[uwb_dev.device_name].header = uwb_dev.header
                    self.device_dict[uwb_dev.device_name].position = uwb_dev.point


if __name__ == "__main__":
    rospy.init_node("light_room_device_visualizer")
    visualizer = LightRoomDeviceVisualizer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer._publish_marker()
        rate.sleep()
