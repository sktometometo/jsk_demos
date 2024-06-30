#!/usr/bin/env python

import copy
import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rospy
import yaml
from geometry_msgs.msg import Point
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from std_msgs.msg import Header, String
from uwb_localization.msg import SDPUWBDeviceArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class DeviceInfo:
    device_name: str
    header: Header = Header()
    position: Point = Point()
    status: Optional[bool] = None

    def is_complete(self) -> bool:
        return all(
            [
                self.status is not None,
                self.header.frame_id != "",
            ]
        )


class LightRoomDeviceVisualizer:

    def __init__(self):

        self._pub_marker = rospy.Publisher(
            "/lock_device_demo_marker_array",
            MarkerArray,
            queue_size=1,
        )
        self._pub_pictogram = rospy.Publisher(
            "/lock_device_demo_pictogram_array",
            PictogramArray,
            queue_size=1,
        )

        self.device_dict: Dict[str, DeviceInfo] = dict()
        self.lock_device_dict = threading.Lock()

        self._interface = UWBSDPInterface()
        self._interface.register_interface_callback(
            ("Key status", "?"),
            self._callback_key_status,
        )

        self._sub_uwb_device = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._callback_uwb_device,
        )

    def _callback_key_status(self, address: Tuple, data_frame: DataFrame):
        device_interfaces = copy.deepcopy(self._interface.device_interfaces)
        if address not in device_interfaces:
            return
        else:
            device_name = device_interfaces[address]["device_name"]
            status = data_frame.content[0]
            with self.lock_device_dict:
                if device_name not in self.device_dict:
                    self.device_dict[device_name] = DeviceInfo(
                        device_name=device_name,
                        status=status,
                    )
                else:
                    self.device_dict[device_name].status = status

    def _callback_uwb_device(self, msg: SDPUWBDeviceArray):
        device_interfaces = copy.deepcopy(self._interface.device_interfaces)
        dev_dict = {
            dev_if["device_name"]: dev_if["interfaces"]
            for addr, dev_if in device_interfaces.items()
        }
        for uwb_dev in msg.devices:
            if uwb_dev.device_name not in dev_dict:
                print(f"Device {uwb_dev.device_name} not in device_interfaces")
                continue
            if ("Key control", "s") not in dev_dict[uwb_dev.device_name]:
                print(
                    f"Device {uwb_dev.device_name} does not have key status interface: {dev_dict[uwb_dev.device_name]}"
                )
                continue
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

    def _publish_marker(self):

        marker_array = MarkerArray()
        pictogram_array = PictogramArray()
        with self.lock_device_dict:
            for device_name, device_info in self.device_dict.items():
                if not device_info.is_complete():
                    show = False
                else:
                    show = True
                #
                # Pin marker
                #
                marker = Marker()
                marker.header = device_info.header
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
                marker.color.r = 25 / 255.0
                marker.color.g = 255 / 255.0
                marker.color.b = 240 / 255.0
                marker_array.markers.append(marker)
                #
                marker = Marker()
                marker.header = device_info.header
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
                marker.color.r = 25 / 255.0
                marker.color.g = 255 / 255.0
                marker.color.b = 240 / 255.0
                marker_array.markers.append(marker)
                #
                # Device name marker
                #
                marker = Marker()
                marker.header = device_info.header
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
                #
                # Lock Status Pictogram
                #
                pictogram = Pictogram()
                pictogram.header = device_info.header
                pictogram.pose.position.x = device_info.position.x
                pictogram.pose.position.y = device_info.position.y - 0.5
                pictogram.pose.position.z = 1.5
                pictogram.pose.orientation.y = -1.0 / math.sqrt(2)
                pictogram.pose.orientation.w = 1.0 / math.sqrt(2)
                pictogram.mode = Pictogram.PICTOGRAM_MODE
                pictogram.speed = 1.0
                pictogram.size = 1.0
                pictogram.color.a = 1.0
                if device_info.status:
                    pictogram.color.r = 1.0
                    pictogram.color.g = 0
                    pictogram.color.b = 0
                    pictogram.character = "lock"
                else:
                    pictogram.color.r = 0.0
                    pictogram.color.g = 1.0
                    pictogram.color.b = 0.0
                    pictogram.character = "lock-open"
                pictogram_array.pictograms.append(pictogram)
                pictogram_array.header = device_info.header
        self._pub_marker.publish(marker_array)
        self._pub_pictogram.publish(pictogram_array)


if __name__ == "__main__":
    rospy.init_node("light_room_device_visualizer")
    visualizer = LightRoomDeviceVisualizer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer._publish_marker()
        print(visualizer.device_dict)
        rate.sleep()
