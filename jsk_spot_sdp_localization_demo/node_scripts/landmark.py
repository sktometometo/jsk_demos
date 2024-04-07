#!/usr/bin/env python

import copy
import math
import threading
from typing import Dict, List, Optional

import PyKDL
import rospy
from jsk_spot_utils.look_at_client import SpotLookAtClient
from nav_msgs.msg import Odometry
from ros_lock import ROSLock, roslock_acquire
from smart_device_protocol.smart_device_protocol_interface import UWBSDPInterface
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray


class Demo:

    def __init__(self):

        self._threshold_distance_max = rospy.get_param("~threshold_distance_max", 30.0)
        self._duration_deadzone = rospy.get_param("~duration_deadzone", 60.0)

        self._spot_client = SpotRosClient()
        self._look_at_client = SpotLookAtClient()
        self._sdp_interface = UWBSDPInterface()
        self._sound_client = SoundClient()

        self._sdp_interface.register_interface_callback(
            ("Landmark information", "S"),
            self._cb_sdp_landmark_information,
        )

        self._lock_for_mobility = ROSLock("mobility")

        self._frame_vision_to_body: Optional[PyKDL.Frame] = None
        self._last_stamps: Dict[str, rospy.Time] = {}

        self._landmark_information_tables_lock = threading.Lock()
        self._landmark_information_tables: Dict[str, str] = {}

        self._sub_odom = rospy.Subscriber("~odom", Odometry, self._cb_odom)

    def point_and_describe(self, target_frame_robotbased, name="", description=""):

        with roslock_acquire(self._lock_for_mobility):
            self._spot_client.trajectory(
                0,
                0,
                math.atan2(target_frame_robotbased.p[1], target_frame_robotbased.p[0]),
                blocking=True,
            )
            self._look_at_client(target_frame_robotbased.p)
            self._sound_client.say(f"There is {name} there.", blocking=True)
            self._sound_client.say(description, blocking=True)

    def _cb_sdp_landmark_information(self, src_address, contents: List):
        device_content = contents[0]
        if src_address not in self._sdp_interface.device_interfaces:
            rospy.logerr(f"Failed to find device interface for {src_address}")
            return
        device_name = self._sdp_interface.device_interfaces[src_address]["device_name"]
        with self._landmark_information_tables_lock:
            self._landmark_information_tables[device_name] = device_content

    def _cb_odom(self, msg: Odometry):

        self._frame_vision_to_body = PyKDL.Frame(
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
        if len(msg.devices) == 0 or self._frame_vision_to_body is None:
            return

        target_device = sorted(
            msg.devices,
            key=lambda x: (
                PyKDL.Vector(x.point.x, x.point.y, x.point.z)
                - self._frame_vision_to_body.p
            ).Norm(),
        )[0]
        last_stamp = self._last_stamps.get(target_device.device_name, None)
        target_frame_robotbased = self._frame_vision_to_body.Inverse() * PyKDL.Frame(
            PyKDL.Rotation(),
            PyKDL.Vector(
                target_device.point.x, target_device.point.y, target_device.point.z
            ),
        )
        with self._landmark_information_tables_lock:
            if target_device.device_name not in self._landmark_information_tables:
                return
            device_description = self._landmark_information_tables[
                target_device.device_name
            ]
        device_interfaces = copy.deepcopy(self._sdp_interface.device_interfaces)
        if target_device.device_name not in [
            di["device_name"] for di in device_interfaces.values()
        ]:
            rospy.logerr(
                f"Failed to find device interface for {target_device.device_name}"
            )
            return
        src_address = [
            k
            for k, v in device_interfaces.items()
            if v["device_name"] == target_device.device_name
        ][0]
        distance = device_interfaces[src_address]["distance"]

        if distance > self._threshold_distance_max:
            rospy.logerr(f"Distance {distance} is too far.")
            return

        if last_stamp is None or rospy.Time.now() - last_stamp > rospy.Duration(
            self._duration_deadzone
        ):
            self._last_stamps[target_device.device_name] = rospy.Time.now()
            with self._landmark_information_tables_lock:
                device_description = self._landmark_information_tables.get(
                    target_device.device_name, ""
                )
            self.point_and_describe(
                target_frame_robotbased,
                name=target_device.device_name,
                description=device_description,
            )


if __name__ == "__main__":
    rospy.init_node("landmark")
    demo = Demo()
    rospy.spin()
