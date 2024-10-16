#!/usr/bin/env python3

import copy
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import PyKDL
import rospy
import yaml
from jsk_recognition_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry
from ros_lock import ROSLock, roslock_acquire
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from sound_play.libsoundplay import SoundClient
from spot_msgs.msg import GraphNavLocalization
from spot_ros_client.libspotros import SpotRosClient
from std_msgs.msg import String
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray
from std_srvs.srv import Trigger, TriggerRequest


@dataclass
class TableEntry:
    device_name: str
    address: Tuple
    nearest_node_id: Optional[str] = None
    nearest_distance: Optional[float] = None


class LightRoomDemo:

    def __init__(self):

        self._reset_localize = rospy.ServiceProxy("/device_localization_node/reset", Trigger)

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sound_client = SoundClient()
        self._sdp_interface = UWBSDPInterface()

        self._detected_face_id = ("Detected face", "s")

        self._target_person_name = "shinjo"
        self._trigger = False
        self._trigger_known = False
        self._triggered_device_addr = None

        self._current_waypoint = ""

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._device_table_lock = threading.Lock()
        self._device_table: Dict[Tuple, TableEntry] = {}

        self._pub_debug_string = rospy.Publisher(
            "/debug_string",
            String,
            queue_size=1,
        )

        self._thread_debug = threading.Thread(target=self._publish_debug)
        self._thread_debug.start()

        self._sub_graph_nav_localization = rospy.Subscriber(
            "/spot/graph_nav_localization_state",
            GraphNavLocalization,
            self._cb_graph_nav_localization,
        )
        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)

        print("initialized")

    @property
    def odom_to_base(self):
        with self._lock_odom:
            return copy.deepcopy(self._odom_to_base)

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

    def _publish_debug(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            message_yaml = ""
            if self._triggered_device_addr is None:
                message_yaml = yaml.dump(
                        {},
                        )
            else:
                with self._device_table_lock:
                    message_yaml = yaml.dump(
                        {
                            "device_neareset_node": self._device_table[
                                self._triggered_device_addr
                            ],
                            "trigger_known": self._trigger_known,
                            "trigger": self._trigger,
                            "address": self._triggered_device_addr,
                        },
                    )
            self._pub_debug_string.publish(String(data=message_yaml))

    def _cb_sdp(self, src_address, data_frame: DataFrame):
        person_name = data_frame.content[0]
        rospy.loginfo("person_detection: {}".format(person_name))
        if not self._trigger:
            self._triggered_device_addr = src_address
            self._trigger = True
            if person_name == self._target_person_name:
                self._trigger_known = True
            else:
                self._trigger_known = False

    def _cb_graph_nav_localization(self, msg: GraphNavLocalization):
        self._current_waypoint = msg.waypoint_id

    def demo(self):
        default_7f_walk_path = "/home/spot/default_7f.walk"
        patrol_id = "leafed-gnu-ZMct0zR8OTzXyQbC0RBKpw=="
        dock_id = "fanned-craw-1uaBLxMUEpHL+J6M3F+F1w=="

        input("Start from 73B2")

        self._client.upload_graph(default_7f_walk_path)
        self._client.set_localization_fiducial()
        self._reset_localize(TriggerRequest())

        # Walk and find a nearest node to each camera
        self._client.navigate_to(
            patrol_id, velocity_limit=(0.3, 0.3, 0.5), blocking=False
        )
        rospy.logwarn("Start")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._client.wait_for_navigate_to_result(duration=rospy.Duration(1.0)):
                break
            rate.sleep()
            dis = self._interface.device_interfaces
            current_node = self._current_waypoint
            with self._device_table_lock:
                for address, entry in dis.items():
                    device_name = entry["device_name"]
                    distance = entry["distance"]
                    if address not in self._device_table.keys():
                        self._device_table[address] = TableEntry(
                            address=address, device_name=device_name
                        )
                    if distance is not None:
                        if (
                            self._device_table[address].nearest_distance is None
                            or distance < self._device_table[address].nearest_distance
                        ):
                            self._device_table[address].nearest_distance = distance
                            self._device_table[address].nearest_node_id = current_node

        print("result: {}".format(self._device_table))
        self._client.navigate_to(dock_id, blocking=True)
        self._client.dock(521)

        self._sdp_interface.register_interface_callback(
            self._detected_face_id,
            self._cb_sdp,
        )
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self._trigger:
            rate.sleep()
        self._client.undock()
        # self._client.navigate_to("agaze-kiwi-zUicg.Z+5tsonwp8LKEdAQ==", blocking=True)
        entry = self._device_table[self._triggered_device_addr]
        print(f"target: {entry}")
        self._client.navigate_to(entry.nearest_node_id, blocking=True)

        # Find person and bring it to 73B2
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(
                "/spot/tracked_world_objects",
                BoundingBoxArray,
            )
            if len(msg.boxes) > 0:
                break
        person_point_kdl = self.odom_to_base.Inverse() * PyKDL.Vector(
            msg.boxes[0].pose.position.x,
            msg.boxes[0].pose.position.y,
            msg.boxes[0].pose.position.z,
        )
        rospy.loginfo("person_point_kdl: {}".format(person_point_kdl))
        self._client.unstow_arm()
        self._client.look_at(
            [person_point_kdl.x(), person_point_kdl.y(), person_point_kdl.z() + 1.5],
            blocking=True,
        )
        if self._trigger_known:
            self._sound_client.say("Wellcome back to JSK laboratory.", blocking=True)
        else:
            self._sound_client.say(
                "Wellcome to JSK laboratory. Follow me.", blocking=True
            )

        self._client.stow_arm()
        self._client.navigate_to(dock_id, blocking=True)
        rospy.logwarn("End")


if __name__ == "__main__":
    rospy.init_node("light_room_demo")
    dummy = rospy.get_param('~dummy', False)
    node = LightRoomDemo()
    if dummy:
        node._sdp_interface.register_interface_callback(
            node._detected_face_id,
            node._cb_sdp,
        )
        rospy.spin()
    else:
        node.demo()
