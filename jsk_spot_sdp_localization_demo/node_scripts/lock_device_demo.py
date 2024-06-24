#!/usr/bin/env python3

import copy
import math
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import PyKDL
import rospy
import tf2_geometry_msgs
import tf2_ros
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, TriggerRequest
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray


@dataclass
class TableEntry:
    device_name: str
    address: Optional[Tuple] = None
    header: Optional[Header] = None
    point: Optional[Point] = None
    lock_status: Optional[bool] = None


class Demo:

    def __init__(self):

        self._reset_localize = rospy.ServiceProxy(
            "/device_localization_node/reset", Trigger
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.spot_client = SpotRosClient()
        self.sound_client = SoundClient()

        self.device_dict: Dict[str, TableEntry] = {}
        self.lock_device_dict = threading.Lock()

        self.interface = UWBSDPInterface(
            callbacks_data={("Key status", "s"): self.key_status_callback}
        )

        self._frame_odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_frame_odom_to_base = threading.Lock()
        self._frame_id_odom: Optional[str] = None
        self._sub_odom = rospy.Subscriber(
            "/spot/odometry",
            Odometry,
            self._cb_odom,
        )

        self._sub_uwb_device = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._callback_uwb_device,
        )

    @property
    def frame_odom_to_base(self) -> PyKDL.Frame:
        rate = rospy.Rate(1)
        while True:
            with self._lock_frame_odom_to_base:
                if self._frame_odom_to_base is not None:
                    return copy.deepcopy(self._frame_odom_to_base)
            rate.sleep()

    @property
    def frame_id_odom(self) -> str:
        rate = rospy.Rate(1)
        while True:
            with self._lock_frame_odom_to_base:
                if self._frame_id_odom is not None:
                    return copy.deepcopy(self._frame_id_odom)
            rate.sleep()

    def _cb_odom(self, msg: Odometry):
        with self._lock_frame_odom_to_base:
            self._frame_id_odom = msg.header.frame_id
            self._frame_odom_to_base = PyKDL.Frame(
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

    def _look_up_transform(
        self, target_frame_id: str, base_frame_id: str = "base_link"
    ) -> Optional[PyKDL.Frame]:

        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame_id,
                base_frame_id,
                rospy.Time(),
            )
            return tf2_geometry_msgs.transform_to_kdl(trans)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def key_status_callback(self, src_address, frame):

        device_interfaces = copy.deepcopy(self.interface.device_interfaces)
        if src_address in device_interfaces:
            status = frame.content[0]
            device_name = device_interfaces[src_address]["device_name"]
            self.device_dict[device_name].lock_status = (
                False if status == "unlocked" else True
            )

    def _callback_uwb_device(self, msg: SDPUWBDeviceArray):

        for uwb_dev in msg.devices:
            assert uwb_dev.header.frame_id == self.frame_id_odom
            with self.lock_device_dict:
                if uwb_dev.device_name not in self.device_dict:
                    self.device_dict[uwb_dev.device_name] = TableEntry(
                        header=uwb_dev.header,
                        point=uwb_dev.point,
                        device_name=uwb_dev.device_name,
                    )
                else:
                    self.device_dict[uwb_dev.device_name].header = uwb_dev.header
                    self.device_dict[uwb_dev.device_name].point = uwb_dev.point

        with self.lock_device_dict:
            for device_name, device_info in self.device_dict.items():
                if device_name not in [uwb_dev.device_name for uwb_dev in msg.devices]:
                    device_info.header = None
                    device_info.point = None

    def control_key(self, lock: bool, device_name: str, timeout: float = 10.0) -> bool:

        self.interface.send(
            device_name,
            DataFrame(
                packet_description="Key control",
                serialization_format="?",
                content=[lock],
            ),
        )
        rate = rospy.Rate(1.0)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rate.sleep()
            if (
                device_name in self.device_dict
                and self.device_dict[device_name].lock_status != lock
            ):
                return True
        return False

    def get_devices_from_direction(
        self,
        target_waypoint_id: str,
        threshold_direction_angle: float = 0.5,
    ) -> List[str]:
        frame_base_to_waypoint = self._look_up_transform(
            "waypoint_" + target_waypoint_id
        )
        if frame_base_to_waypoint is None:
            return []
        points_base_to_devices: Dict[str, PyKDL.Vector] = {
            device_name: self.frame_odom_to_base.Inverse()
            * PyKDL.Vector(
                x=device_info.point.x, y=device_info.point.y, z=device_info.point.z
            )
            for device_name, device_info in self.device_dict.items()
            if device_info.point is not None
        }
        return [
            device_name
            for device_name, point in points_base_to_devices.items()
            if frame_base_to_waypoint.p.Unit().Dot(point.Unit())
            > math.cos(threshold_direction_angle)
        ]

    def run_demo(self):

        graph_path = "/home/spot/default_7f_with_door.walk"
        waypoint_id_73B2_door_inside = "yonder-adder-cjebDHNMdwNaax8EdVqs0A=="
        waypoint_id_breeze_way = "holy-puffin-dfM.pGS6xCB4m190VUNPWw=="
        waypoint_id_73B2_door_outside = ""

        device_name_73B2_door_lock = "SDP Lock 73B2"

        input("Press Enter to start demo. Please make sure that the robot is in 73B2")

        self.spot_client.upload_graph(graph_path)
        self.spot_client.set_localization_fiducial()
        self.spot_client.navigate_to(waypoint_id_73B2_door_inside, blocking=True)

        self.spot_client.navigate_to(waypoint_id_73B2_door_outside, blocking=True)
        self.control_key(True, device_name_73B2_door_lock)

        rospy.loginfo("Moving to breezeway")
        self.spot_client.navigate_to(waypoint_id_breeze_way, blocking=True)

        self._reset_localize(TriggerRequest())
        rospy.logwarn("Demo started")

        rospy.loginfo("Moving back to 73B2")
        self.spot_client.navigate_to(waypoint_id_73B2_door_outside, blocking=True)

        target_waypoint = waypoint_id_73B2_door_outside
        target_devices = self.get_devices_from_direction(target_waypoint)
        if len(target_devices) == 0:
            rospy.logwarn("No devices found in the direction of target waypoint")
            return
        device_name = target_devices[0]
        rospy.logwarn("Target device name is : {}".format(device_name))
        self.control_key(False, device_name)

        self.spot_client.navigate_to(waypoint_id_73B2_door_inside, blocking=True)


if __name__ == "__main__":

    rospy.init_node("demo")
    dummy = rospy.get_param("~dummy", False)
    demo = Demo()
    if dummy:
        rospy.spin()
    else:
        demo.run_demo()
