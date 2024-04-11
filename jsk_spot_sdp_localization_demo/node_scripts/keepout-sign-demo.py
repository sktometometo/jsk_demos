#!/usr/bin/env python3

import copy
import threading
from typing import Optional

import PyKDL
import rospy
from nav_msgs.msg import Odometry
from ros_lock import ROSLock
from smart_device_protocol.smart_device_protocol_interface import UWBSDPInterface
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDeviceArray


class KeepoutSignExecutor:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sdp_interface = UWBSDPInterface()
        self._lock_mobility = ROSLock("mobility")

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._nearest_keepout_sign_dist: Optional[float] = None
        self._lock_nearest_keepout_sign_dist = threading.Lock()

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices", SDPUWBDeviceArray, self._cb_device
        )

    @property
    def odom_to_base(self):
        with self._lock_odom:
            return copy.deepcopy(self._odom_to_base)

    @property
    def nearest_keepout_sign_dist(self):
        with self._lock_nearest_keepout_sign_dist:
            return self._nearest_keepout_sign_dist

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

    def _cb_device(self, msg: SDPUWBDeviceArray):
        if len(msg.devices) == 0:
            rospy.logdebug('lengh of meessag is zero. skpped')
            return

        with self._lock_odom:
            if self.odom_to_base is None:
                return

        dev_ifs = copy.deepcopy(self._sdp_interface.device_interfaces)

        valid_devices = [
                    msg_dev
                    for msg_dev in msg.devices
                    if msg_dev.device_name in dev_ifs
                    and dev_ifs[msg_dev.device_name]["distance"] is not None
                    and 'KeepoutSign' in msg_dev.device_name
                ]
        if len(valid_devices) == 0:
            rospy.logdebug('lengh of valid devices is zero. skpped')
            return

        target_device = sorted(
                valid_devices,
                key=lambda x: dev_ifs[x.device_name]["distance"]
                )[0]

        with self._lock_nearest_keepout_sign_dist:
            self._nearest_keepout_sign_dist = dev_ifs[target_device.device_name]["distance"]


    def run(self):

        rospy.loginfo("First, move the robot to the mai hall of eng2.")
        input("Press Enter > ")
        ret = self._client.upload_graph("/home/spot/default.walk")
        print(f"ret: {ret}")
        ret = self._client.set_localization_fiducial()
        print(f"ret: {ret}")
        input("Move to the elevator hall. Press Enter > ")
        ret = self._client.navigate_to("daft-fleece-OI80B3zjIRf0G4nxz5YLwA==", blocking=True)
        print(f"ret: {ret}")

        self._client.gripper_open()
        input("Give me document. Press Enter > ")
        self._client.gripper_close()

        rospy.loginfo("Move to Reppincan")
        ret = self._client.navigate_to("staple-finch-GrUHQoxOBscHvFykmFcrkQ==", blocking=True)
        print(f"ret: {ret}")


        input("Give me document. Press Enter > ")
        self._client.gripper_open()

        self._client.navigate_to("daft-fleece-OI80B3zjIRf0G4nxz5YLwA==", blocking=False)
        while not rospy.is_shutdown():
            if self._client.wait_for_navigate_to_result(duration=rospy.Duration(1)):
                return
            if self._nearest_keepout_sign_dist < 5.0:
                self._client.cancel_navigate_to()
                break

        self._client.navigate_to("alike-tapir-5izZBNcxp+fG+UmYKLniNQ==", blocking=True)


if __name__ == '__main__':
    rospy.init_node('hoge')
    node = KeepoutSignExecutor()
    node.run()
