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
WAYPOINT_7F_ELEVATOR_OUTSIDE = "filial-hydra-Uw+zYz54tA.Sw07vySSrsQ=="
WAYPOINT_7F_ELEVATOR_INSIDE = "warped-oxen-ZirzngBcug7glDDiKZCgYQ=="

WALK_DIR_PATH_OUTSIDE = "/home/spot/autowalks/20241010_2f_to_outside.walk"
WAYPOINT_OUTSIDE_OUTSIDE_BUILDING = "pet-bonobo-HEBZBWJ61eHEUL2NsPfSlg=="

TARGET_LIST = {
    "73B2 inside": WAYPOINT_7F_73B2_INSIDE,
    "73B2 front outside": WAYPOINT_7F_73B2_OUTSIDE,
    "7F entrance hall": WAYPOINT_7F_ELEVATOR_OUTSIDE,
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
                "Get on the car": self.ride_on_elevator,
                "Get off the car": self.ride_off_elevator,
            }
        )

    def ride_on_elevator(self) -> None:
        rospy.logwarn("Riding on elevator")
        self.spot_client.navigate_to(WAYPOINT_7F_ELEVATOR_INSIDE)

    def ride_off_elevator(self) -> None:
        rospy.logwarn("Riding off elevator")
        self.spot_client.navigate_to(WAYPOINT_7F_ELEVATOR_OUTSIDE)
        self.spot_client.upload_graph(WALK_DIR_PATH_OUTSIDE)
        self.spot_client.set_localization_fiducial()

    def run_demo(self):
        # Testing
        time.sleep(5.)
        #self.call_api("Move to the out of eng. 2 building")
        #self.call_api("Move to the entrance hall of 7F")
        # Demo
        #self.call_api("Move to the front of 73B2")
        #self.call_api("Move to the entrance hall of 7F")
        self.call_api("Call elevator to downstairs")
        while True:
            status = self.call_api("Get the door status")
            print(f"door status: {status}")
            if isinstance(status, tuple) and len(status) > 0 and status[0] is True:
                break
        self.call_api("Get on the elevator car")
        self.call_api("Press the elevator internal panel to 2F")
        while True:
            current_floor = self.call_api("Get the current floor")
            print(f"current floor: {current_floor}")
            if isinstance(current_floor, tuple) and len(current_floor) > 0 and current_floor[0] == 2:
                break
        self.call_api("Get off the elevator car")
        self.call_api("Move to the out of eng. 2 building")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    input("Press Enter to start the demo")
    demo.run_demo()
