#!/usr/bin/env python3

import copy
import math
import threading
from operator import is_
from typing import Dict, List, Optional
import time

import PyKDL
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_spot_lib.look_at_client import SpotLookAtClient
from nav_msgs.msg import Odometry
from ros_lock import ROSLock, roslock_acquire
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from uwb_localization.msg import SDPUWBDevice, SDPUWBDeviceArray
from std_srvs.srv import Trigger, TriggerRequest


class LightRoomDemo:

    def __init__(self):

        self._interface = UWBSDPInterface()
        self._client = SpotRosClient()
        self._sound_client = SoundClient()
        self._sdp_interface = UWBSDPInterface()

        self._reset_localize = rospy.ServiceProxy("/device_localization_node/reset", Trigger)

        self._sdp_interface.register_interface_callback(
            ("Light status", "?"),
            self._cb_sdp,
        )

        self._vel_limit = (0.5, 0.5, 0.3)

        self._turn_on_distance_threshold: float = 3.0

        self._odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_odom = threading.Lock()

        self._odom_to_people: List[PyKDL.Frame] = []
        self._lock_people = threading.Lock()

        self._light_status_table_lock = threading.Lock()
        self._light_status_table: Dict[str, bool] = {}

        self._sub_odom = rospy.Subscriber("/spot/odometry", Odometry, self._cb_odom)
        self._sub_people_bbox = rospy.Subscriber(
            "/spot_recognition/bbox_array",
            BoundingBoxArray,
            self._cb_people_bbox,
        )

        self._running_demo_thread = True
        self._thread_demo = threading.Thread(target=self.demo)
        self._thread_demo.start()

        print("initialized")

    @property
    def device_interfaces(self):
        return copy.deepcopy(self._interface.device_interfaces)

    @property
    def odom_to_base(self):
        with self._lock_odom:
            return copy.deepcopy(self._odom_to_base)

    @property
    def odom_to_people(self):
        with self._lock_people:
            return copy.deepcopy(self._odom_to_people)

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

    def _cb_sdp(self, src_address, data_frame: DataFrame):
        device_content = data_frame.content[0]
        device_interfaces = self.device_interfaces
        if src_address not in device_interfaces:
            rospy.logerr(f"Failed to find device interface for {src_address}")
            return
        device_name = device_interfaces[src_address]["device_name"]
        with self._light_status_table_lock:
            self._light_status_table[device_name] = device_content

    def _cb_people_bbox(self, msg: BoundingBoxArray):
        with self._lock_people:
            self._odom_to_people = [
                PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        msg_box.pose.orientation.x,
                        msg_box.pose.orientation.y,
                        msg_box.pose.orientation.z,
                        msg_box.pose.orientation.w,
                    ),
                    PyKDL.Vector(
                        msg_box.pose.position.x,
                        msg_box.pose.position.y,
                        msg_box.pose.position.z,
                    ),
                )
                for msg_box in msg.boxes
            ]

    def turn_light(self, device_name: str, status: str, timeout: float = 20.):
        print("send frame")
        self._sdp_interface.send(
            device_name,
            DataFrame(
                packet_description="Light control",
                content=[status],
            ),
        )
        self._sdp_interface.send(
            device_name,
            DataFrame(
                packet_description="Light control",
                content=[status],
            ),
        )
        self._sdp_interface.send(
            device_name,
            DataFrame(
                packet_description="Light control",
                content=[status],
            ),
        )
        print("sent frame")
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rospy.sleep(1.0)
            try:
                print(f"Waiting for status updated: {self._light_status_table[device_name]}")
                if self._light_status_table[device_name] == status:
                    break
            except KeyError:
                pass

    def no_people_behind(self):
        odom_to_base = self.odom_to_base
        return (
            len(
                [
                    odom_to_base.Inverse() * person_frame
                    for person_frame in self.odom_to_people
                    if (odom_to_base.Inverse() * person_frame).p[0] < 0
                ]
            )
            == 0
        )

    def no_people_around(self):
        return len(self.odom_to_people) == 0

    def stop_demo(self):
        self._running_demo_thread = False
        self._thread_demo.join()

    def demo(self):
        #self.is_out = True
        self.is_out = {}
        rospy.logwarn('Demo Started')
        while not rospy.is_shutdown() and self._running_demo_thread:
            rospy.sleep(1)
            with self._light_status_table_lock:
                rospy.loginfo("==========================")
                rospy.loginfo(f"is_out: {self.is_out}")
                for device_name, status in self._light_status_table.items():
                    device_interfaces = self.device_interfaces
                    for addr, dev_if in device_interfaces.items():
                        if dev_if["device_name"] == device_name:
                            if device_name not in self.is_out:
                                self.is_out[device_name] = True
                            try:
                                distance = self._sdp_interface.device_interfaces[addr]["distance"]
                            except KeyError:
                                distance = None
                            rospy.loginfo( "  =======================")
                            rospy.loginfo(f"  device_name: {device_name}")
                            rospy.loginfo(f"  distance: {distance}")
                            rospy.loginfo(f"  status: {status}")
                            rospy.loginfo(f"  is_out: {self.is_out[device_name]}")
                            rospy.loginfo( "  =======================")
                            if distance is None:
                                rospy.loginfo("Distance is none")
                                continue
                            if distance < self._turn_on_distance_threshold and self.is_out[device_name]:
                                self.is_out[device_name] = False
                                rospy.logwarn(f"is_out[{device_name}] -> False")
                                if not status:
                                    rospy.logwarn(f"turn on to {device_name}")
                                    self.turn_light(device_name, True)
                            elif distance >= self._turn_on_distance_threshold and not self.is_out[device_name]:
                                self.is_out[device_name] = True
                                rospy.loginfo(f"is_out[{device_name}] -> True")
                                if status:
                                    pass
                                    #rospy.logwarn(f"turn off to {device_name}")
                                    #self.turn_light(device_name, False)
                            else:
                                rospy.loginfo("skipped")
                rospy.loginfo("==========================")
        rospy.logwarn('Demo stopped')

    def walk(self):
        default_7f_walk_path = '/home/spot/default_7f_with_door.walk'
        target_id_73b1 = 'chief-iguana-rCeWvuq9uAAhdq7D+trwMw=='
        target_id_73b2 = 'yonder-adder-cjebDHNMdwNaax8EdVqs0A=='
        target_id_73a4 = 'soured-cocoon-KRBT4IqkmBwCauxpgDhFEQ=='
        start_id = 'holy-puffin-dfM.pGS6xCB4m190VUNPWw=='
        goal_id = 'unsold-shrew-.KEEBLDLzp+zG8MLyAWU.Q=='

        self._client.upload_graph(default_7f_walk_path)
        success, message = self._client.set_localization_fiducial()
        if not success:
            rospy.logerror("Failed to localize. Aborting.")
            return

        self._client.navigate_to(start_id, blocking=True)
        self._reset_localize(TriggerRequest())

        rospy.logwarn('Start')
        success, message = self._client.navigate_to(target_id_73b1, velocity_limit=self._vel_limit, blocking=True)

        # 73B1
        if success:
            time.sleep(10.)
            no_people_around = self.no_people_around()
            if not no_people_around:
                rospy.logwarn("There is people in the room. So I will warn them and leave")
                self._sound_client.say("Please make sure to lock all doors and windows before leaving the house")
            else:
                rospy.logwarn("No people around. So turn off the light")
                self.turn_light("SDP Switchbot 73B1", False)
            self._client.navigate_to(target_id_73b2, velocity_limit=self._vel_limit, blocking=False)
        else:
            rospy.logwarn("Failed to reach 73B1")
            self._client.navigate_to(target_id_73b2, velocity_limit=self._vel_limit, blocking=False)
        self._client.wait_for_navigate_to_result()
        success, message = self._client.get_navigate_to_result()


        # 73B2
        if success:
            time.sleep(10.)
            no_people_around = self.no_people_around()
            if not no_people_around:
                rospy.logwarn("There is people in the room. So I will warn them and leave")
                self._sound_client.say("Please make sure to lock all doors and windows before leaving the house")
            else:
                rospy.logwarn("No people around. So turn off the light")
                self.turn_light("SDP Switchbot", False)
            self._client.navigate_to(target_id_73a4, velocity_limit=self._vel_limit, blocking=False)
        else:
            rospy.logwarn("Failed to reach 73B2")
            self._client.navigate_to(target_id_73a4, velocity_limit=self._vel_limit, blocking=False)
        self._client.wait_for_navigate_to_result()
        success, message = self._client.get_navigate_to_result()

        # 73A4
        if success:
            time.sleep(10.)
            no_people_around = self.no_people_around()
            if not no_people_around:
                rospy.logwarn("There is people in the room. So I will warn them and leave")
                self._sound_client.say("Please make sure to lock all doors and windows before leaving the house")
            else:
                rospy.logwarn("No people around. So turn off the light")
                self.turn_light("SDP Switchbot 73A4", False)
            self._client.navigate_to(start_id, velocity_limit=self._vel_limit, blocking=False)
        else:
            rospy.logwarn("Failed to reach 73A4")
            self._client.navigate_to(start_id, velocity_limit=self._vel_limit, blocking=False)
        self._client.wait_for_navigate_to_result()
        success, message = self._client.get_navigate_to_result()

        self._client.navigate_to(goal_id, blocking=True)
        rospy.logwarn('End')


if __name__ == "__main__":
    rospy.init_node("light_room_demo")
    do_walk = rospy.get_param('~do_walk', True)
    node = LightRoomDemo()
    if do_walk:
        node.walk()
    else:
        rospy.spin()
    node.stop_demo()
