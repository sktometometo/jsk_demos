#!/usr/bin/env python

import argparse
import copy
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import rospy
import yaml
from autonomous_integration.active_api_discovery import (
    ActiveAPIDiscovery,
    cosine_similarity,
)
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import *
from autonomous_integration.sdp_utils import (
    API_TYPE,
    SDPType,
    convert_type_string_to_format_char,
    get_arguments_list_from_function,
    get_response_list_from_function,
)
from autonomous_integration.spot_auto_int_demo import SpotAutoIntegDemo
from openai_ros.srv import Embedding, EmbeddingRequest
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


TARGET_LIST = {
    "73A4_INSIDE": WAYPOINT_73A4_INSIDE,
    "73A4_FRONT_OUTSIDE": WAYPOINT_73A4_OUTSIDE,
    "73B1_INSIDE": WAYPOINT_73B1_INSIDE,
    "73B1_FRONT_OUTSIDE": WAYPOINT_73B1_OUTSIDE,
    "73B2_INSIDE": WAYPOINT_73B2_INSIDE,
    "73B2_FRONT_OUTSIDE": WAYPOINT_73B2_OUTSIDE,
}


class Demo(SpotAutoIntegDemo):

    def __init__(self, waypoint_target_list: Dict[str, str] = TARGET_LIST):
        super().__init__(waypoint_target_list=waypoint_target_list)

    def init_demo(
        self,
        walk_path: str = DEFAULT_WALK_PATH,
    ):
        self.spot_client.upload_graph(walk_path)
        self.spot_client.set_localization_fiducial()

    def run_demo(
        self,
        room_name: str = "73A4",
    ):

        self.call_api("Move to the front of {}".format(room_name))
        self.call_api("Unlock the key.")
        self.call_api("Move into {}".format(room_name))
        self.call_api("Turn on the light.")
        res = self.call_api("Get the number of people")
        rospy.logerr("Number of people: %s", res)
        self.call_api('Say "There are people in the room"')
        # self.call_api("Turn off the light.")
        self.call_api("Move out of {}".format(room_name))
        # self.call_api("Lock the key of the room.")


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
        demo.init_demo()
    input("Press Enter to start the demo")
    demo.run_demo(args.target)
