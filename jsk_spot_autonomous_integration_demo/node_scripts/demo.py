#!/usr/bin/env python

import argparse
from typing import Callable, Dict, List, Optional

import rospy
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import *
from spot_demo import SpotDemo


class Demo(SpotDemo):

    def __init__(self, api_key: str):
        super().__init__()
        self.discovery = ActiveAPIDiscovery(api_key)
        self.completion = ArgumentCompletion(api_key)

    def get_people(self) -> List:
        return self._odom_to_people

    def run_demo(self):

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

        device_name_73B2_door_lock = "SDP Lock 73B2"

        self.spot_client.upload_graph(default_7f_walk_path)
        self.spot_client.set_localization_fiducial()

        # Enter 73B2
        self.spot_client.navigate_to(waypoint_id_73B2_door_inside, blocking=True)

        # Turn on light
        intension = "Turn on the light in the room."
        api_full_list = get_api_list(self.sdp_interface)
        api_short_list = [
            (api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list
        ]
        target_api_short = self.discovery.select_api(intension, {}, [], api_short_list)
        target_api_full = api_full_list[api_short_list.index(target_api_short)]
        target_api_args = self.completion.generate_arguments_for_api(
            intension,
            {},
            [],
            target_api_short[0],
            target_api_short[1],
            target_api_short[2],
        )
        call_api(self.sdp_interface, target_api_full, target_api_args)

        if len(self.get_people()) > 0:
            self.sound_client.say("There are people in the room")
        else:
            call_from_intension(
                self.sdp_interface,
                self.discovery,
                self.completion,
                "Turn off the light in the room.",
                {},
                [],
            )

        # Exit room
        self.spot_client.navigate_to(waypoint_id_73B2_door_outside, blocking=True)

        call_from_intension(
            self.sdp_interface,
            self.discovery,
            self.completion,
            "Lock the key of the room.",
            {},
            [],
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo(args.api_key)
    demo.run_demo()
