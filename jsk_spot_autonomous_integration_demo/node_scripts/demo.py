#!/usr/bin/env python

import time
import argparse
from typing import Callable, Dict, List, Optional, Tuple

import rospy
import yaml
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import *
from spot_demo import SpotDemo
from std_msgs.msg import String

default_7f_walk_path = "/home/spot/default_7f_with_door.walk"
target_id_73b1 = "chief-iguana-rCeWvuq9uAAhdq7D+trwMw=="
target_id_73b2 = "yonder-adder-cjebDHNMdwNaax8EdVqs0A=="
target_id_73a4 = "soured-cocoon-KRBT4IqkmBwCauxpgDhFEQ=="
start_id = "holy-puffin-dfM.pGS6xCB4m190VUNPWw=="
goal_id = "unsold-shrew-.KEEBLDLzp+zG8MLyAWU.Q=="
waypoint_id_73B2_door_inside = "yonder-adder-cjebDHNMdwNaax8EdVqs0A=="
waypoint_id_breeze_way = "holy-puffin-dfM.pGS6xCB4m190VUNPWw=="
waypoint_id_73B2_door_outside = "deific-toad-MtBK4mWxiBSG4N8xF7rYDg=="
waypoint_id_73A4_door_outside = "ivied-mamba-exgntSAsmmo.715WHYfo7w=="


def convert_names_and_types_to_string(names_and_types: List[Tuple[str, str]]) -> str:
    return yaml.dump({name: type for name, type in names_and_types})


def convert_api_type_to_string(api_type: API_TYPE) -> str:
    return yaml.dump(
        {
            "address": list(api_type[0]),
            "device_name": api_type[1],
            "type": api_type[2].value,
            "description": api_type[3],
            "serialization_format": api_type[4],
            "argument_names_and_types": convert_names_and_types_to_string(api_type[5]),
            "response_names_and_types": convert_names_and_types_to_string(api_type[6]),
        }
    )


def convert_api_type_list_to_string(api_type_list: List[API_TYPE]) -> str:
    return yaml.dump([convert_api_type_to_string(api) for api in api_type_list])


class Demo(SpotDemo):

    def __init__(self):
        super().__init__()
        self.discovery = ActiveAPIDiscovery()
        self.completion = ArgumentCompletion()

        self.pub_debug_string = rospy.Publisher(
            "/debug_string",
            String,
            queue_size=1,
        )

    def get_people(self) -> List:
        return self._odom_to_people

    def init_demo(
        self,
        walk_path: str = default_7f_walk_path,
        dummy: bool = False,
    ):
        if not dummy:
            self.spot_client.upload_graph(walk_path)
            self.spot_client.set_localization_fiducial()

    def run_demo(
        self,
        walk_path: str = default_7f_walk_path,
        waypoint_id_door_inside: str = waypoint_id_73B2_door_inside,
        waypoint_id_door_outside: str = waypoint_id_73B2_door_outside,
        dummy: bool = False,
    ):

        # Enter 73B2
        if not dummy:
            self.spot_client.navigate_to(waypoint_id_door_inside, blocking=True)

        #
        # Turn on light
        #
        intension = "Turn on the light in the room."
        api_full_list = get_api_list(self.sdp_interface)
        rospy.loginfo(f"api_full_list: {api_full_list}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "api_full_list",
                        "data": convert_api_type_list_to_string(api_full_list),
                    }
                )
            )
        )
        api_short_list = [
            (api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list
        ]

        target_api_short = self.discovery.select_api(intension, {}, [], api_short_list)
        target_api_full = api_full_list[api_short_list.index(target_api_short)]
        rospy.loginfo(f"target_api: {target_api_short}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "target_api_selection",
                        "data": {
                            "api": convert_api_type_to_string(target_api_full),
                            "intension": intension,
                            "arguments": {},
                            "response_names_and_types": [],
                        },
                    }
                )
            )
        )
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
                            "api": convert_api_type_to_string(target_api_full),
                            "arguments": target_api_args,
                        },
                    }
                )
            )
        )
        if not dummy:
            call_api(self.sdp_interface, target_api_full, target_api_args)

        if len(self.get_people()) > 0:
            if not dummy:
                self.sound_client.say("There are people in the room")
        else:
            #
            # Turn off light
            #
            intension = "Turn off the light in the room."
            api_full_list = get_api_list(self.sdp_interface)
            rospy.loginfo(f"api_full_list: {api_full_list}")
            self.pub_debug_string.publish(
                String(
                    data=yaml.dump(
                        {
                            "string_type": "api_full_list",
                            "data": convert_api_type_list_to_string(api_full_list),
                        }
                    )
                )
            )
            api_short_list = [
                (api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list
            ]

            target_api_short = self.discovery.select_api(
                intension, {}, [], api_short_list
            )
            target_api_full = api_full_list[api_short_list.index(target_api_short)]
            rospy.loginfo(f"target_api: {target_api_short}")
            self.pub_debug_string.publish(
                String(
                    data=yaml.dump(
                        {
                            "string_type": "target_api_selection",
                            "data": {
                                "api": convert_api_type_to_string(target_api_full),
                                "intension": intension,
                                "arguments": {},
                                "response_names_and_types": [],
                            },
                        }
                    )
                )
            )
            target_api_args = self.completion.generate_arguments_for_api(
                intension,
                {},
                [],
                target_api_short[0],
                target_api_short[1],
                target_api_short[2],
            )
            rospy.loginfo(
                f"api_calling: api: {target_api_short}, args: {target_api_args}"
            )
            self.pub_debug_string.publish(
                String(
                    data=yaml.dump(
                        {
                            "string_type": "api_call",
                            "data": {
                                "api": convert_api_type_to_string(target_api_full),
                                "arguments": target_api_args,
                            },
                        }
                    )
                )
            )
            if not dummy:
                call_api(self.sdp_interface, target_api_full, target_api_args)

        # Exit room
        if not dummy:
            self.spot_client.navigate_to(waypoint_id_door_outside, blocking=True)

        #
        # Lock the key
        #
        intension = "Lock the key of the room."
        api_full_list = get_api_list(self.sdp_interface)
        rospy.loginfo(f"api_full_list: {api_full_list}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "api_full_list",
                        "data": convert_api_type_list_to_string(api_full_list),
                    }
                )
            )
        )
        api_short_list = [
            (api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list
        ]
        target_api_short = self.discovery.select_api(intension, {}, [], api_short_list)
        target_api_full = api_full_list[api_short_list.index(target_api_short)]
        rospy.loginfo(f"target_api: {target_api_short}")
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": "target_api_selection",
                        "data": {
                            "api": convert_api_type_to_string(target_api_full),
                            "intension": intension,
                            "arguments": {},
                            "response_names_and_types": [],
                        },
                    }
                )
            )
        )
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
                            "api": convert_api_type_to_string(target_api_full),
                            "arguments": target_api_args,
                        },
                    }
                )
            )
        )
        if not dummy:
            call_api(self.sdp_interface, target_api_full, target_api_args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dummy", action="store_true")
    parser.add_argument("--init", action="store_true")
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    time.sleep(5.0)
    if args.init:
        demo.init_demo(dummy=args.dummy)
    input("Press Enter to start the demo")
    demo.run_demo(dummy=args.dummy)
