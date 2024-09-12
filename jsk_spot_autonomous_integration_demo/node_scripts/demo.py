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

DEFAULT_WALK_PATH = "/home/spot/default_7f_with_door.walk"
WAYPOINT_BREEZEWAY = "holy-puffin-dfM.pGS6xCB4m190VUNPWw=="
WAYPOINT_73B1_INSIDE = "chief-iguana-rCeWvuq9uAAhdq7D+trwMw=="
WAYPOINT_73B1_OUTSIDE = "teary-buck-AyEZ13ivnObxX2Rn2dgLIg=="
WAYPOINT_73B2_INSIDE = "yonder-adder-cjebDHNMdwNaax8EdVqs0A=="
WAYPOINT_73B2_OUTSIDE = "deific-toad-MtBK4mWxiBSG4N8xF7rYDg=="
WAYPOINT_73A4_INSIDE = "ivied-mamba-exgntSAsmmo.715WHYfo7w=="
WAYPOINT_73A4_OUTSIDE = "soured-cocoon-KRBT4IqkmBwCauxpgDhFEQ=="


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
        walk_path: str = DEFAULT_WALK_PATH,
        dummy: bool = False,
    ):
        if not dummy:
            self.spot_client.upload_graph(walk_path)
            self.spot_client.set_localization_fiducial()

    def run_demo(
        self,
        waypoint_id_door_inside: str = WAYPOINT_73B2_INSIDE,
        waypoint_id_door_outside: str = WAYPOINT_73B2_OUTSIDE,
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
    parser.add_argument("--target", choices=["73A4", "73B1", "73B2"], default="73B2")
    parser.add_argument("--dummy", action="store_true")
    parser.add_argument("--init", action="store_true")
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    time.sleep(5.0)
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
