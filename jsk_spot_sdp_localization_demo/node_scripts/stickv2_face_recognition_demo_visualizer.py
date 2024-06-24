#!/usr/bin/env python

import copy
import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rospy
import yaml
from geometry_msgs.msg import Point
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from std_msgs.msg import Header, String
from uwb_localization.msg import SDPUWBDeviceArray
from visualization_msgs.msg import Marker, MarkerArray
from yaml.constructor import ConstructorError


@dataclass
class FaceRecognitionDeviceInfo:
    device_name: str
    waypoint_id: Optional[str] = None
    header: Optional[Header] = None
    point: Optional[Point] = None
    detected_stamp: rospy.Time = rospy.Time(0)
    detected_person: Optional[str] = None

    def is_complete(self) -> bool:
        return all(
            [
                self.header is not None,
                self.point is not None,
            ]
        )

    @property
    def detected(self) -> bool:
        return (
            self.detected_person is not None
            and (rospy.Time.now() - self.detected_stamp).to_sec() < 3.0
        )


class SafeLoaderWithTuple(yaml.SafeLoader):
    pass


def construct_tuple(loader, node):
    try:
        return tuple(loader.construct_sequence(node))
    except ConstructorError as e:
        raise ConstructorError(
            "could not determine a constructor for the tag 'tag:yaml.org,2002:python/tuple'",
            e.problem_mark,
        )


def construct_table_entry(loader, node):
    node = loader.construct_mapping(node)
    return node


SafeLoaderWithTuple.add_constructor("tag:yaml.org,2002:python/tuple", construct_tuple)
SafeLoaderWithTuple.add_constructor(
    "tag:yaml.org,2002:python/object:__main__.TableEntry", construct_table_entry
)


class FaceRecognitionVisualizer:

    def __init__(self):

        self._pub_marker = rospy.Publisher(
            "/face_recognition_marker_array",
            MarkerArray,
            queue_size=1,
        )
        self._pub_detected_string = rospy.Publisher(
            "/face_recognition_detected",
            String,
            queue_size=1,
        )

        self._sdp_interface = UWBSDPInterface()
        self._sdp_interface.register_interface_callback(
            ("Detected face", "s"), self._cb_cam_sdp
        )

        self.device_dict: Dict[str, FaceRecognitionDeviceInfo] = {}
        self.lock_device_dict = threading.Lock()

        self._sub_debug = rospy.Subscriber(
            "/debug_string",
            String,
            self._cb_debug,
        )

        self._sub_uwb_device = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._callback_uwb_device,
        )

    def _cb_cam_sdp(self, address: Tuple, data_frame: DataFrame):
        device_interfaces = copy.deepcopy(self._sdp_interface.device_interfaces)
        if address in device_interfaces:
            device_name = device_interfaces[address]["device_name"]
            with self.lock_device_dict:
                if device_name not in self.device_dict:
                    self.device_dict[device_name] = FaceRecognitionDeviceInfo(
                        device_name=device_name,
                        detected_person=data_frame.content[0],
                        detected_stamp=rospy.Time.now(),
                    )
                else:
                    self.device_dict[device_name].detected_person = data_frame.content[
                        0
                    ]
                    self.device_dict[device_name].detected_stamp = rospy.Time.now()

    def _cb_debug(self, msg: String):
        data = yaml.load(msg.data, Loader=SafeLoaderWithTuple)
        if "device_neareset_node" in data:
            device_name = data["device_neareset_node"]["device_name"]
            nearest_node_id = data["device_neareset_node"]["nearest_node_id"]
            with self.lock_device_dict:
                if device_name not in self.device_dict:
                    self.device_dict[device_name] = FaceRecognitionDeviceInfo(
                        device_name=device_name,
                        waypoint_id=nearest_node_id,
                    )
                else:
                    self.device_dict[device_name].waypoint_id = nearest_node_id

    def _callback_uwb_device(self, msg: SDPUWBDeviceArray):

        device_interfaces = copy.deepcopy(self._sdp_interface.device_interfaces)
        device_interfaces = {
            dev_inf["device_name"]: dev_inf
            for addr, dev_inf in device_interfaces.items()
        }
        for uwb_dev in msg.devices:
            if (
                uwb_dev.device_name in device_interfaces
                and ("Target mode", "s")
                in device_interfaces[uwb_dev.device_name]["interfaces"]
            ):
                with self.lock_device_dict:
                    if uwb_dev.device_name not in self.device_dict:
                        self.device_dict[uwb_dev.device_name] = (
                            FaceRecognitionDeviceInfo(
                                device_name=uwb_dev.device_name,
                                header=uwb_dev.header,
                                point=uwb_dev.point,
                            )
                        )
                    else:
                        self.device_dict[uwb_dev.device_name].header = uwb_dev.header
                        self.device_dict[uwb_dev.device_name].point = uwb_dev.point

        with self.lock_device_dict:
            for device_name, device_info in self.device_dict.items():
                if device_name not in [dev.device_name for dev in msg.devices]:
                    device_info.header = None
                    device_info.point = None

    def _publish_marker(self):

        marker_array = MarkerArray()
        published_string = False
        with self.lock_device_dict:
            print(self.device_dict)
            for device_name, device_info in self.device_dict.items():
                #
                # Show device position marker
                #
                if not device_info.is_complete():
                    show_device_position = False
                else:
                    show_device_position = True
                marker = Marker()
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD if show_device_position else Marker.DELETE
                if show_device_position:
                    marker.header = device_info.header
                    marker.pose.position.x = device_info.point.x
                    marker.pose.position.y = device_info.point.y
                    marker.pose.position.z = 0.75
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 1.5
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                marker_array.markers.append(marker)
                marker = Marker()
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364 + 1
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD if show_device_position else Marker.DELETE
                if show_device_position:
                    marker.header = device_info.header
                    marker.pose.position.x = device_info.point.x
                    marker.pose.position.y = device_info.point.y
                    marker.pose.position.z = 1.5
                    marker.pose.orientation.y = 1.0 / math.sqrt(2)
                    marker.pose.orientation.w = 1.0 / math.sqrt(2)
                    marker.scale.x = 0.5
                    marker.scale.y = 0.5
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                if device_info.detected:
                    show_detected_text = True
                else:
                    show_detected_text = False
                marker_array.markers.append(marker)

                marker_array.markers.append(marker)
                marker = Marker()
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364 + 2
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD if show_device_position else Marker.DELETE
                if show_device_position:
                    marker.header = device_info.header
                    marker.pose.position.x = device_info.point.x
                    marker.pose.position.y = device_info.point.y
                    marker.pose.position.z = 2.0
                    marker.pose.orientation.w = 1.0
                    marker.scale.z = 0.5
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    if show_detected_text:
                        marker.text = "Detected: " + device_info.detected_person
                        self._pub_detected_string.publish(
                            String(
                                data="Detected: "
                                + device_info.detected_person
                                + " at "
                                + device_info.device_name
                            )
                        )
                        published_string = True
                    else:
                        marker.text = "No person detected"
                marker_array.markers.append(marker)

                marker = Marker()
                marker.ns = "light_room_device"
                marker.id = hash(device_name) % 214748364 + 3
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD if show_device_position else Marker.DELETE
                if show_device_position:
                    marker.header = device_info.header
                    marker.pose.position.x = device_info.point.x
                    marker.pose.position.y = device_info.point.y
                    marker.pose.position.z = 2.5
                    marker.pose.orientation.w = 1.0
                    marker.scale.z = 0.5
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.text = device_name
                marker_array.markers.append(marker)

        self._pub_marker.publish(marker_array)

        if not published_string:
            self._pub_detected_string.publish(
                String(data="No person detected at all devices")
            )


if __name__ == "__main__":
    rospy.init_node("fd_demo_visualizer")
    visualizer = FaceRecognitionVisualizer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer._publish_marker()
        rate.sleep()
