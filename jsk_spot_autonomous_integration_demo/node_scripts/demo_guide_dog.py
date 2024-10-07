#!/usr/bin/env python

import argparse
from typing import Any, Callable, Dict, List, Optional, Tuple

import rospy
from autonomous_integration.sdp_utils import *
from autonomous_integration.spot_auto_int_demo import SpotAutoIntegDemo

WALK_DIR_PATH_7F = "/home/spot/default_7f_with_door.walk"
WAYPOINT_7F_BREEZEWAY = "holy-puffin-dfM.pGS6xCB4m190VUNPWw=="
WAYPOINT_7F_73B1_INSIDE = "chief-iguana-rCeWvuq9uAAhdq7D+trwMw=="
WAYPOINT_7F_73B1_OUTSIDE = "teary-buck-AyEZ13ivnObxX2Rn2dgLIg=="
WAYPOINT_7F_73B2_INSIDE = "yonder-adder-cjebDHNMdwNaax8EdVqs0A=="
WAYPOINT_7F_73B2_OUTSIDE = "deific-toad-MtBK4mWxiBSG4N8xF7rYDg=="
WAYPOINT_7F_73A4_INSIDE = "soured-cocoon-KRBT4IqkmBwCauxpgDhFEQ=="
WAYPOINT_7F_73A4_OUTSIDE = "ivied-mamba-exgntSAsmmo.715WHYfo7w=="
WAYPOINT_7F_ELEVATOR_OUTSIDE = "ivied-mamba-exgntSAsmmo.715WHYfo7w=="
WAYPOINT_7F_ELEVATOR_INSIDE = "ivied-mamba-exgntSAsmmo.715WHYfo7w=="

WALK_DIR_PATH_OUTSIDE = ""
WAYPOINT_OUTSIDE_ELEVATOR_OUTSIDE = ""
WAYPOINT_OUTSIDE_ELEVATOR_INSIDE = ""
WAYPOINT_OUTSIDE_OUTSIDE_BUILDING = ""

TARGET_LIST = {
    "73B2 inside": WAYPOINT_7F_73B2_INSIDE,
    "73B2 front outside": WAYPOINT_7F_73B2_OUTSIDE,
    "7F elevator outside": WAYPOINT_7F_ELEVATOR_OUTSIDE,
    "7F elevator inside": WAYPOINT_7F_ELEVATOR_INSIDE,
    "outside of building": WAYPOINT_OUTSIDE_OUTSIDE_BUILDING,
}


class Demo(SpotAutoIntegDemo):

    def __init__(self, target_list: Dict[str, str] = TARGET_LIST):
        super().__init__(target_list)

        # self._target_api = {
        #     "Speak": self.speak,
        #     'Move to target place, (e.g. "73B2 inside")': self.move_to_target,
        #     "Ride on the elevator": self.ride_on_elevator,
        #     "Ride off the elevator": self.ride_off_elevator,
        # }
        self._target_api.update(
            {
                "Ride on the elevator": self.ride_on_elevator,
                "Ride off the elevator": self.ride_off_elevator,
            }
        )

    def ride_on_elevator(self) -> None:
        self.spot_client.navigate_to(WAYPOINT_7F_ELEVATOR_INSIDE)

    def ride_off_elevator(self) -> None:
        self.spot_client.navigate_to(WAYPOINT_7F_ELEVATOR_OUTSIDE)
        self.spot_client.upload_graph(WALK_DIR_PATH_OUTSIDE)
        self.spot_client.set_localization_fiducial()

    def run_demo(self):
        # Init
        self.spot_client.upload_graph(WALK_DIR_PATH_7F)
        self.spot_client.set_localization_fiducial()

        # Demo
        self.call_api("Move to the front of 73B2")
        self.call_api("Move to the elevator hall at 7F")
        self.call_api("Call elevator to downstairs")
        self.call_api("Ride on the elevator")
        self.call_api("Press the elevator button to 2F")
        self.call_api("Ride off the elevator")
        self.call_api("Move to the out of eng. 2 building")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    input("Press Enter to start the demo")
    demo.run_demo()
