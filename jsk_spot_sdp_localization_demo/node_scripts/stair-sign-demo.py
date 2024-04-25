#!/usr/bin/env python3

import threading
from typing import Dict, List, Optional, Tuple

import PyKDL
import rospy
from ros_lock import ROSLock
from smart_device_protocol import DataFrame, UWBSDPInterface
from spot_localization_demo.stair_demo_util import (
    StairEntry,
    StairPointType,
    get_markers_msg,
)
from spot_ros_client import SpotROSClient
from uwb_localization.msg import SDPUWBDeviceArray
from visualization_msgs.msg import Marker, MarkerArray


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

        self._pub_markers = rospy.Publisher("/stair_markers", MarkerArray, queue_size=1)

        self._interface = UWBSDPInterface()
        self._client = SpotROSClient()
        self._lock_mobility = ROSLock("mobility")

        self._lock_stair_tables = threading.Lock()
        self._stair_tables: Dict[str, StairEntry] = {}
        self._sdp_device_parent_frame_id: Optional[str] = None

        self._sdp_interface = UWBSDPInterface()
        self._sdp_interface.register_interface_callback(
            ("Landmark information", "S"),
            self._cb_sdp_landmark_information,
        )

        self._sub_devices = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._cb_device,
        )

        self._thread_marker_publish = threading.Thread(target=self.marker_publish_spin)
        self._thread_marker_publish.start()

    @property
    def stair_tables(self):
        with self._lock_stair_tables:
            return self._stair_tables

    def _cb_sdp_landmark_information(self, src_address, data_frame: DataFrame):
        device_content = data_frame.content[0]
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

    def _cb_device(self, msg: SDPUWBDeviceArray):
        with self._lock_stair_tables:
            for msg_device in msg.devices:
                if self._sdp_device_parent_frame_id is None:
                    self._sdp_device_parent_frame_id = msg_device.header.frame_id
                device_name = msg_device.device_name
                for stair_entry in self._stair_tables.values():
                    if stair_entry.stair_top_a_device_name == device_name:
                        stair_entry.frame_id = msg_device.header.frame_id
                        stair_entry.stair_top_point_a = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )
                    elif stair_entry.stair_top_b_device_name == device_name:
                        stair_entry.frame_id = msg_device.header.frame_id
                        stair_entry.stair_top_point_b = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )
                    elif stair_entry.stair_bottom_a_device_name == device_name:
                        stair_entry.frame_id = msg_device.header.frame_id
                        stair_entry.stair_bottom_point_a = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )
                    elif stair_entry.stair_bottom_b_device_name == device_name:
                        stair_entry.frame_id = msg_device.header.frame_id
                        stair_entry.stair_bottom_point_b = PyKDL.Vector(
                            msg_device.position.x,
                            msg_device.position.y,
                            msg_device.position.z,
                        )

    def marker_publish_spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self._lock_stair_tables:
                msg = get_markers_msg(
                    self._stair_tables, self._sdp_device_parent_frame_id
                )
                self._pub_markers.publish(msg)
            rate.sleep()

    def run(self):

        print("First, move the robot to the 2F entrance of eng2.")
        input("Press Enter > ")
        ret = self._client.upload_graph("/home/spot/default.walk")
        print(f"ret: {ret}")

        ret = self._client.set_localization_fiducial()
        print(f"ret: {ret}")

        rospy.loginfo("Move to Main Gate")
        ret = self._client.navigate_to(
            "cymose-fish-gX2bZc.jOqWFufA1BiarmA==", blocking=True
        )

        # DO some interaction

        carry_bag = True

        print("Move to Eng. 2")
        self._client.navigate_to(
            "curtal-mouse-NIEKemfgciiPnRV7uOVN9A==", blocking=False
        )
        while not rospy.is_shutdown():
            if self._client.wait_for_navigate_to_result(duration=rospy.Duration(1)):
                return

            if carry_bag and False:
                self._client.cancel_navigate_to()
                self._client.navigate_to(
                    "umteen-fleece-W97IMsdJRTH4y6b5dJWANA==", blocking=True
                )
                return


if __name__ == "__main__":
    rospy.init_node("stair_executor")
    executor = StairExecutor()
    executor.run()
