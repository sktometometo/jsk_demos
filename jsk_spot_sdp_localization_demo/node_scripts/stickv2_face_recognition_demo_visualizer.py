#!/usr/bin/env python

import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

import rospy
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class FaceRecognitionInfo:
    device_name: str
    waypoint_id: str
    detected: bool = False

    def is_complete(self) -> bool:
        return True


class FaceRecognitionVisualizer:

    def __init__(self):

        self._pub_marker = rospy.Publisher(
            "/face_recognition_marker_array",
            MarkerArray,
            queue_size=1,
        )

        self.device_dict: Dict[str, FaceRecognitionInfo] = dict()
        self.lock_device_dict = threading.Lock()

    def _publish_marker(self):

        marker_array = MarkerArray()
        with self.lock_device_dict:
            for waypoint_id, device_info in self.device_dict.items():
                if not device_info.is_complete():
                    show = False
                else:
                    show = True
                #
                marker = Marker()
                marker.header = device_info.waypoint_id
                marker.ns = "light_room_device"
                marker.id = hash(waypoint_id) % 214748364
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD if show else Marker.DELETE
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
                marker.header = device_info.waypoint_id
                marker.ns = "light_room_device"
                marker.id = hash(waypoint_id) % 214748364 + 1
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD if show else Marker.DELETE
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
