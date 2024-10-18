#!/usr/bin/env python

import copy
import math
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rospy
import yaml
from geometry_msgs.msg import Point
from jsk_rviz_plugins.msg import OverlayText, Pictogram, PictogramArray
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from std_msgs.msg import Header, String
from uwb_localization.msg import SDPUWBDeviceArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class LightDeviceInfo:
    device_name: str
    header: Header = Header()
    position: Point = Point()
    distance: Optional[float] = None
    status: Optional[bool] = None

    def is_complete(self) -> bool:
        return all(
            [
                self.distance is not None and self.distance <= 5.0,
                self.status is not None,
                self.header.frame_id != "",
            ]
        )


@dataclass
class LockDeviceInfo:
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


class DebugVisualizer:

    def __init__(self):
        self.sdp_interface = UWBSDPInterface()
        self.sdp_interface.register_interface_callback(
            ("Light status", "?"), self._callback_light_interface
        )
        self.sdp_interface.register_interface_callback(
            ("Key status", "?"),
            self._callback_key_status,
        )

        self._pub_marker_light = rospy.Publisher(
            "/light_room_device_marker_array",
            MarkerArray,
            queue_size=1,
        )
        self._pub_marker_lock = rospy.Publisher(
            "/lock_device_demo_marker_array",
            MarkerArray,
            queue_size=1,
        )
        self._pub_pictogram_lock = rospy.Publisher(
            "/lock_device_demo_pictogram_array",
            PictogramArray,
            queue_size=1,
        )

        self.lock_device_dict: Dict[str, LockDeviceInfo] = dict()
        self.lock_lock_device_dict = threading.Lock()

        self.light_device_dict: Dict[str, LightDeviceInfo] = dict()
        self.lock_light_device_dict = threading.Lock()

        self._pub_api_list = rospy.Publisher(
            "/debug_text_overlay/api_list", OverlayText, queue_size=1
        )
        self._pub_target_api = rospy.Publisher(
            "/debug_text_overlay/target_api", OverlayText, queue_size=1
        )
        self._pub_api_call = rospy.Publisher(
            "/debug_text_overlay/api_call", OverlayText, queue_size=1
        )

        self.timeout_api_list = rospy.Duration(
            float(rospy.get_param("~timeout_api_list", "10.0"))
        )
        self.timeout_target_api = rospy.Duration(
            float(rospy.get_param("~timeout_target_api", "10.0"))
        )
        self.timeout_api_call = rospy.Duration(
            float(rospy.get_param("~timeout_api_call", "10.0"))
        )

        self._api_list: List[Tuple[str, str, List[str]]] = []
        self._api_list_stamp = rospy.Time.now()
        self._api_list_lock = threading.Lock()
        self._target_api: List[Tuple[str, str]] = []
        self._target_api_intension = ""
        self._target_api_stamp = rospy.Time.now()
        self._target_api_lock = threading.Lock()
        self._api_call: Optional[Tuple[str, str, Dict[str, Any]]] = None
        self._api_call_stamp = rospy.Time.now()
        self._api_call_lock = threading.Lock()

        self._sub = rospy.Subscriber("/debug_string", String, self._callback)
        self._sub_uwb_device = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._callback_uwb_device,
        )

    def _callback_light_interface(self, address: Tuple, frame):
        rospy.logerr(f"address: {address}, frame: {frame}")
        device_interfaces = self.sdp_interface.device_interfaces
        if address not in device_interfaces:
            return
        device_interface = device_interfaces[address]
        device_name = device_interface["device_name"]
        distance = device_interface["distance"]
        status = frame.content[0]
        with self.lock_light_device_dict:
            if device_name not in self.light_device_dict:
                self.light_device_dict[device_name] = LightDeviceInfo(
                    device_name=device_name,
                    distance=distance,
                    status=status,
                )
            else:
                self.light_device_dict[device_name].distance = distance
                self.light_device_dict[device_name].status = status

    def _callback_key_status(self, address: Tuple, data_frame: DataFrame):
        device_interfaces = copy.deepcopy(self.sdp_interface.device_interfaces)
        if address not in device_interfaces:
            return
        else:
            device_name = device_interfaces[address]["device_name"]
            status = data_frame.content[0]
            with self.lock_lock_device_dict:
                if device_name not in self.lock_device_dict:
                    self.lock_device_dict[device_name] = LockDeviceInfo(
                        device_name=device_name,
                        status=status,
                    )
                else:
                    self.lock_device_dict[device_name].status = status

    def _callback_uwb_device(self, msg: SDPUWBDeviceArray):
        rospy.logerr(f"msg: {msg}")
        device_interfaces = copy.deepcopy(self.sdp_interface.device_interfaces)
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
            with self.lock_lock_device_dict:
                if uwb_dev.device_name not in self.lock_device_dict:
                    self.lock_device_dict[uwb_dev.device_name] = LockDeviceInfo(
                        header=uwb_dev.header,
                        position=uwb_dev.point,
                        device_name=uwb_dev.device_name,
                    )
                else:
                    self.lock_device_dict[uwb_dev.device_name].header = uwb_dev.header
                    self.lock_device_dict[uwb_dev.device_name].position = uwb_dev.point

        for uwb_dev in msg.devices:
            with self.lock_light_device_dict:
                if uwb_dev.device_name not in self.light_device_dict:
                    self.light_device_dict[uwb_dev.device_name] = LightDeviceInfo(
                        header=uwb_dev.header,
                        position=uwb_dev.point,
                        device_name=uwb_dev.device_name,
                    )
                else:
                    self.light_device_dict[uwb_dev.device_name].header = uwb_dev.header
                    self.light_device_dict[uwb_dev.device_name].position = uwb_dev.point

    def _callback(self, msg):

        data = yaml.load(msg.data)
        if data["string_type"] == "api_full_list":
            api_list = data["data"]
            with self._api_list_lock:
                self._api_list = [
                    (
                        api["device_name"],
                        api["description"],
                        [
                            name
                            for name, type in api["argument_names_and_types"].items()
                        ],
                    )
                    for api in api_list
                ]
                self._api_list_stamp = rospy.Time.now()
        elif data["string_type"] == "target_api_selection":
            # target_api_list = data["data"]["target_apis"]
            # intension = data["data"]["intension"]
            with self._target_api_lock:
                self._target_api = [
                    (api["api"]["device_name"], api["api"]["description"])
                    for api in data["data"]
                ]
                self._target_api_intension = (
                    data["data"][0]["intension"] if len(data["data"]) > 0 else ""
                )
                self._target_api_stamp = rospy.Time.now()
        elif data["string_type"] == "api_call":
            print(f"data['data']: {data['data']}")
            api = data["data"]["api"]
            arguments = data["data"]["arguments"]
            api_device_name = api["device_name"]
            api_description = api["description"]
            with self._api_call_lock:
                self._api_call = (api_device_name, api_description, arguments)
                self._api_call_stamp = rospy.Time.now()
        else:
            rospy.logwarn("Invalid string type")
            return

    def _publish_api_call(self):

        with self._api_call_lock:
            if rospy.Time.now() - self._api_call_stamp > self.timeout_api_call:
                return
            if self._api_call is None:
                return
            api_device_name, api_description, arguments = self._api_call
            text = OverlayText()
            text.action = OverlayText.ADD
            text.width = 400
            text.height = 600
            text.left = 10
            text.top = 10
            text.bg_color.a = 0.2
            text.line_width = 2
            text.text_size = 12
            text.font = "DejaVu Sans Mono"
            text.fg_color.r = 1.0
            text.fg_color.g = 1.0
            text.fg_color.b = 1.0
            text.fg_color.a = 1.0
            text.text = f"API call: {api_device_name}\n{api_description}\n"
            for name, value in arguments.items():
                text.text += f"\t{name}: {value}\n"
            self._pub_api_call.publish(text)

    def _publish_api_list(self):

        with self._api_list_lock:
            if rospy.Time.now() - self._api_list_stamp > self.timeout_api_list:
                return
            text = OverlayText()
            text.action = OverlayText.ADD
            text.width = 400
            text.height = 600
            text.left = 10
            text.top = 10
            text.bg_color.a = 0.2
            text.line_width = 2
            text.text_size = 12
            text.font = "DejaVu Sans Mono"
            text.fg_color.r = 1.0
            text.fg_color.g = 1.0
            text.fg_color.b = 1.0
            text.fg_color.a = 1.0
            text.text = "API list\n"
            for device_name, description, argument_names in self._api_list:
                text.text += f"== {device_name} ==\n"
                text.text += f"Desc: {description}\n"
                text.text += "Args: {}\n".format(", ".join(argument_names))
            self._pub_api_list.publish(text)

    def _publish_target_api(self):

        with self._target_api_lock:
            if rospy.Time.now() - self._target_api_stamp > self.timeout_target_api:
                return
            text = OverlayText()
            text.action = OverlayText.ADD
            text.width = 400
            text.height = 600
            text.left = 10
            text.top = 10
            text.bg_color.a = 0.2
            text.line_width = 2
            text.text_size = 12
            text.font = "DejaVu Sans Mono"
            text.fg_color.r = 1.0
            text.fg_color.g = 1.0
            text.fg_color.b = 1.0
            text.fg_color.a = 1.0
            text.text = f"Target API\nIntension: {self._target_api_intension}\n"
            for device_name, description in self._target_api:
                text.text += f"{device_name}: {description}\n"
            self._pub_target_api.publish(text)

    def _publish_marker_light(self):

        marker_array = MarkerArray()
        with self.lock_light_device_dict:
            for device_name, device_info in self.light_device_dict.items():
                if not device_info.is_complete():
                    show = False
                    rospy.logerr(f"Device {device_name} is not complete")
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
                if False:
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
                if False:
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
        self._pub_marker_light.publish(marker_array)

    def _publish_marker_lock(self):

        marker_array = MarkerArray()
        pictogram_array = PictogramArray()
        with self.lock_lock_device_dict:
            for device_name, device_info in self.lock_device_dict.items():
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
        self._pub_marker_lock.publish(marker_array)
        self._pub_pictogram_lock.publish(pictogram_array)

    def run(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._publish_api_list()
            self._publish_target_api()
            self._publish_api_call()
            self._publish_marker_light()
            self._publish_marker_lock()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("debug_visualizer")
    visualizer = DebugVisualizer()
    visualizer.run()
