#!/usr/bin/env python

import rospy
import PyKDL
from typing import Optional

from ros_lock import ROSLock, roslock_acquire

from jsk_spot_utils.look_at_client import SpotLookAtClient
from spot_ros_client.libspotros import SpotRosClient
from smart_device_protocol.smart_device_protocol_interface import UWBSDPInterface
from sound_play.libsoundplay import SoundClient

from nav_msgs.msg import Odometry
from uwb_localization.msg ipmort SDPUWBDeviceArray



class Demo:

    def __init__(self):

        self._spot_client = SpotRosClient()
        self._look_at_client = SpotLookAtClient()
        self._sdp_interface = UWBSDPInterface()
        self._sound_client = SoundClient()

        self._frame_vision_to_body: Optional[PyKDL.Frame] = None

        self._lock_for_mobility = ROSLock("mobility")

        self._sub_odom = rospy.Subscriber(
                "~odom",
                Odometry,
                self._cb_odom
                )

    def point_and_describe(self, target_frame_robotbased, name="", description=""):

        with roslock_acquire(self._lock_for_mobility):
            self._spot_client.trajectory(
                        0, 0, math.atan2(
                            target_frame_robotbased.p[1],
                            target_frame_robotbased.p[0]
                            ),
                        blocking=True
                    )
            self._look_at_client(target_frame_robotbased.p)
            self._sound_client.say(f"There is {name} there.", blocking=True)
            self._sound_client.say(description, blocking=True)

    def _cb_odom(self, msg: Odometry)

        self._frame_vision_to_body = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                    ),
                PyKDL.Vector(
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                    )
                )

    def _cb_device(self, msg: SDPUWBDeviceArray):
        if len(msg.devices) == 0:
            return

        sorted(msg.devices, key=)
