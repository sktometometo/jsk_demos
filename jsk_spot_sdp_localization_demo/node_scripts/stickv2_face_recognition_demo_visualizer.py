#!/usr/bin/env python

import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

import rospy
import yaml
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from yaml.constructor import ConstructorError


@dataclass
class FaceRecognitionInfo:
    device_name: str
    waypoint_id: str
    detected: bool = False

    def is_complete(self) -> bool:
        return True


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

        self.device_dict: Dict[str, FaceRecognitionInfo] = {}
        self.lock_device_dict = threading.Lock()

        self._sub_debug = rospy.Subscriber(
            "/debug_string",
            String,
            self._cb_debug,
        )

    def _cb_debug(self, msg: String):
        data = yaml.load(msg.data, Loader=SafeLoaderWithTuple)
        print(data)
        if "device_neareset_node" in data:
            with self.lock_device_dict:
                self.device_dict[data["device_neareset_node"]["nearest_node_id"]] = (
                    FaceRecognitionInfo(
                        device_name=data["device_neareset_node"]["device_name"],
                        waypoint_id=data["device_neareset_node"]["nearest_node_id"],
                        detected=True,
                    )
                )

    def _publish_marker(self):

        marker_array = MarkerArray()
        with self.lock_device_dict:
            print(self.device_dict)
            for waypoint_id, device_info in self.device_dict.items():
                #
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "waypoint_" + device_info.waypoint_id
                marker.ns = "light_room_device"
                marker.id = hash(waypoint_id) % 214748364
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = 0
                marker.pose.position.y = 0
                marker.pose.position.z = 0.75
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 1.5
                marker.color.a = 1.0
                if not device_info.detected:
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
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "waypoint_" + device_info.waypoint_id
                marker.ns = "light_room_device"
                marker.id = hash(waypoint_id) % 214748364 + 1
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = 0
                marker.pose.position.y = 0
                marker.pose.position.z = 1.5
                marker.pose.orientation.y = 1.0 / math.sqrt(2)
                marker.pose.orientation.w = 1.0 / math.sqrt(2)
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.1
                marker.color.a = 1.0
                if not device_info.detected:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                marker_array.markers.append(marker)
        self._pub_marker.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("fd_demo_visualizer")
    visualizer = FaceRecognitionVisualizer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer._publish_marker()
        rate.sleep()
