#!/usr/bin/env python

import argparse
import copy
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import rospy
import yaml
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import *
from spot_demo import SpotDemo
from std_msgs.msg import String
from uwb_localization.msg import SDPUWBDeviceArray

DEFAULT_WALK_PATH = "/home/spot/default_7f_with_door.walk"
WAYPOINT_BREEZEWAY = "holy-puffin-dfM.pGS6xCB4m190VUNPWw=="
WAYPOINT_73B1_INSIDE = "chief-iguana-rCeWvuq9uAAhdq7D+trwMw=="
WAYPOINT_73B1_OUTSIDE = "teary-buck-AyEZ13ivnObxX2Rn2dgLIg=="
WAYPOINT_73B2_INSIDE = "yonder-adder-cjebDHNMdwNaax8EdVqs0A=="
WAYPOINT_73B2_OUTSIDE = "deific-toad-MtBK4mWxiBSG4N8xF7rYDg=="
WAYPOINT_73A4_INSIDE = "soured-cocoon-KRBT4IqkmBwCauxpgDhFEQ=="
WAYPOINT_73A4_OUTSIDE = "ivied-mamba-exgntSAsmmo.715WHYfo7w=="


def convert_names_and_types_to_string_ready(
    names_and_types: List[Tuple[str, str]]
) -> Dict[str, str]:
    return {name: type for name, type in names_and_types}


def convert_api_type_to_string_ready(api_type: API_TYPE) -> Dict[str, Any]:
    return {
        "address": list(api_type[0]),
        "device_name": api_type[1],
        "type": api_type[2].value,
        "description": api_type[3],
        "serialization_format": api_type[4],
        "argument_names_and_types": convert_names_and_types_to_string_ready(
            api_type[5]
        ),
        "response_names_and_types": convert_names_and_types_to_string_ready(
            api_type[6]
        ),
    }


def convert_api_type_list_to_string_ready(api_type_list: List[API_TYPE]) -> List:
    return [convert_api_type_to_string_ready(api) for api in api_type_list]


class Demo(SpotDemo):

    def __init__(self):
        super().__init__()
        self.discovery = ActiveAPIDiscovery()
        self.completion = ArgumentCompletion()

        self._lock_sdpuwb = threading.Lock()
        self._sdpuwb = SDPUWBDeviceArray()

        self.pub_debug_string = rospy.Publisher(
            "/debug_string",
            String,
            queue_size=1,
        )
        self._sub_sdpuwb = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._sdpuwb_callback,
        )

    def _sdpuwb_callback(self, msg: SDPUWBDeviceArray):
        with self._lock_sdpuwb:
            self._sdpuwb = msg

    @property
    def sdpuwb_devices(self) -> SDPUWBDeviceArray:
        with self._lock_sdpuwb:
            return copy.deepcopy(self._sdpuwb)

    def get_people(self) -> List:
        return self._odom_to_people

    def init_demo(
        self,
        walk_path: str = DEFAULT_WALK_PATH,
        dummy: bool = False,
    ):
        if not dummy:
            self.spot_client.upload_graph(walk_path)
            self.spot_client.set_localization_fiducial()

    def start_api_list(self):
        self._api_list_loop_running = True
        self._api_list_thread = threading.Thread(target=self._api_list_publish_loop)
        self._api_list_thread.start()

    def stop_api_list(self):
        self._api_list_loop_running = False
        self._api_list_thread.join()

    def _api_list_publish_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and self._api_list_loop_running:
            rate.sleep()
            api_full_list = get_api_list(self.sdp_interface)
            # rospy.loginfo(f"api_full_list: {api_full_list}")
            self.pub_debug_string.publish(
                String(
                    data=yaml.dump(
                        {
                            "string_type": "api_full_list",
                            "data": convert_api_type_list_to_string_ready(
                                api_full_list
                            ),
                        }
                    )
                )
            )

    def call_device(
        self,
        intension: str,
    ) -> Optional[Tuple]:
        #
        # Turn on light
        #
        api_full_list = get_api_list(self.sdp_interface)
        rospy.loginfo(f"api_full_list: {api_full_list}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "api_full_list",
                        "data": convert_api_type_list_to_string_ready(api_full_list),
                    }
                )
            )
        )
        api_short_list = [
            (api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list
        ]
        target_api_list_short_with_similarity = self.discovery.select_api(
            intension, {}, [], api_short_list
        )
        target_api_list_short = [
            target_api_short
            for similarity, target_api_short in target_api_list_short_with_similarity
        ]
        target_api_list_full = [
            api_full_list[api_short_list.index(target_api_short)]
            for target_api_short in target_api_list_short
        ]
        rospy.loginfo(f"target_api: {target_api_list_short}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "target_api_selection",
                        "data": [
                            {
                                "api": convert_api_type_to_string_ready(
                                    target_api_full
                                ),
                                "intension": intension,
                                "arguments": {},
                                "response_names_and_types": [],
                            }
                            for target_api_full in target_api_list_full
                        ],
                    }
                )
            )
        )
        # Get closest target api
        target_api_full = None
        distance_to_base = float("inf")
        device_interfaces = self.sdp_interface.device_interfaces
        rospy.loginfo("device_interfaces: %s", device_interfaces)
        for target_api_full_candidate in target_api_list_full:
            device_name = target_api_full_candidate[1]
            # Get dev_info for device_name from device_interfaces
            dev_info = None
            for addr, dev_if in device_interfaces.items():
                if dev_if["device_name"] == device_name:
                    dev_info = dev_if
                    break
            if dev_info is None:
                rospy.logerr(f"Device {device_name} not found in device_interfaces")
                continue
            if "distance" not in dev_info or dev_info["distance"] is None:
                rospy.logerr(f"Distance for {device_name} not found")
                continue
            distance = dev_info["distance"]
            distance_stamp = dev_info["distance_stamp"]
            if rospy.Time.now() - distance_stamp > rospy.Duration(10.0):
                rospy.logerr(f"Distance for {device_name} is too old: {distance_stamp}")
                continue
            if distance < distance_to_base:
                target_api_full = target_api_full_candidate
                distance_to_base = distance
        if target_api_full is None:
            rospy.logerr("No suitable API found")
            return None
        target_api_short = target_api_list_short[
            target_api_list_full.index(target_api_full)
        ]
        target_api_args = self.completion.generate_arguments_for_api(
            intension,
            {},
            [],
            target_api_short[0],
            target_api_short[1],
            target_api_short[2],
        )
        rospy.loginfo(f"api_calling: api: {target_api_short}, args: {target_api_args}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "api_call",
                        "data": {
                            "api": convert_api_type_to_string_ready(target_api_full),
                            "arguments": target_api_args,
                        },
                    }
                )
            )
        )
        time.sleep(5.0)
        return call_api(self.sdp_interface, target_api_full, target_api_args)

    def run_demo(
        self,
        waypoint_id_door_inside: str = WAYPOINT_73B2_INSIDE,
        waypoint_id_door_outside: str = WAYPOINT_73B2_OUTSIDE,
        dummy: bool = False,
    ):

        #
        if not dummy:
            self.spot_client.navigate_to(waypoint_id_door_outside, blocking=True)
            time.sleep(3.0)

        self.call_device("Unlock the key.")

        # Enter 73B2
        if not dummy:
            self.spot_client.navigate_to(waypoint_id_door_inside, blocking=True)
            time.sleep(3.0)

        self.call_device("Turn on the light.")

        # Say
        self.sound_client.say("There are people in the room")

        # Exit room
        if not dummy:
            self.spot_client.navigate_to(waypoint_id_door_outside, blocking=True)

        # #
        # # Lock the key
        # #
        # self.call_device("Lock the key of the room.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=["73A4", "73B1", "73B2"], default="73B2")
    parser.add_argument("--dummy", action="store_true")
    parser.add_argument("--init", action="store_true")
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    time.sleep(5.0)
    demo.start_api_list()
    if args.init:
        demo.init_demo(dummy=args.dummy)
    input("Press Enter to start the demo")
    if args.target == "73A4":
        demo.run_demo(
            waypoint_id_door_inside=WAYPOINT_73A4_INSIDE,
            waypoint_id_door_outside=WAYPOINT_73A4_OUTSIDE,
            dummy=args.dummy,
        )
    elif args.target == "73B1":
        demo.run_demo(
            waypoint_id_door_inside=WAYPOINT_73B1_INSIDE,
            waypoint_id_door_outside=WAYPOINT_73B1_OUTSIDE,
            dummy=args.dummy,
        )
    elif args.target == "73B2":
        demo.run_demo(
            waypoint_id_door_inside=WAYPOINT_73B2_INSIDE,
            waypoint_id_door_outside=WAYPOINT_73B2_OUTSIDE,
            dummy=args.dummy,
        )
    else:
        raise ValueError(f"Invalid target: {args.target}")
    demo.stop_api_list()
