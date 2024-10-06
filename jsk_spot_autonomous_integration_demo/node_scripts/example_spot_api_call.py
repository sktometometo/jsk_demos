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
    "73A4_OUTSIDE": WAYPOINT_73A4_OUTSIDE,
    "73B1_INSIDE": WAYPOINT_73B1_INSIDE,
    "73B1_OUTSIDE": WAYPOINT_73B1_OUTSIDE,
    "73B2_INSIDE": WAYPOINT_73B2_INSIDE,
    "73B2_OUTSIDE": WAYPOINT_73B2_OUTSIDE,
}


class Demo(SpotDemo):

    def __init__(self):
        super().__init__()
        self.discovery = ActiveAPIDiscovery()
        self.completion = ArgumentCompletion()

        self.get_embedding = rospy.ServiceProxy("/openai/get_embedding", Embedding)

        self.pub_debug_string = rospy.Publisher(
            "/debug_string",
            String,
            queue_size=1,
        )

        self._target_api = {
            "Speak": self.speak,
            "Move to the target place, (e.g. the front of room, the inside of room, the outside of room, etc..)": self.move_to_target,
        }

    def get_spot_api_list(self) -> List[API_TYPE]:
        api_full_list: List[API_TYPE] = []
        for api_description, api in self._target_api.items():
            arguments_names_and_types = get_arguments_list_from_function(api)
            response_names_and_types = get_response_list_from_function(api)
            serialization_format = ""
            for arg_name, arg_type in arguments_names_and_types:
                serialization_format += convert_type_string_to_format_char(arg_type)
            api_full_list.append(
                (
                    (0, 0, 0, 0, 0, 0),
                    "Robot API",
                    SDPType.SPOT,
                    api_description,
                    serialization_format,
                    arguments_names_and_types,
                    response_names_and_types,
                )
            )
        return api_full_list

    def _get_embedding(
        self,
        text: str,
    ) -> np.ndarray:
        """
        Get the embedding of the given text.
        """
        res = self.get_embedding(EmbeddingRequest(prompt=text))
        embeddings = res.embedding
        return np.array(embeddings)

    def _choose_target(self, raw_target: str, threshold: float = 0.5) -> Optional[str]:

        target = None
        target_similarity = -float("inf")
        for target_name, target_waypoint in TARGET_LIST.items():
            similarity = cosine_similarity(
                self._get_embedding(raw_target), self._get_embedding(target_name)
            )
            rospy.loginfo(
                f"Similarity between {raw_target} and {target_name}: {similarity}"
            )
            if similarity > threshold and similarity > target_similarity:
                target = target_name
                target_similarity = similarity
        return target

    def move_to_target(self, target_place: str) -> None:
        target_waypoint_name = self._choose_target(target_place)
        if target_waypoint_name is None:
            rospy.logerr(f"Target not found: {target_place}")
            return
        rospy.loginfo("Navigating to %s", target_waypoint_name)
        self.spot_client.navigate_to(TARGET_LIST[target_waypoint_name], blocking=True)

    def speak(self, text: str) -> None:
        rospy.loginfo(f"Speaking: {text}")
        self.sound_client.say(text)

    def call_spot_api(
        self,
        api_full: API_TYPE,
        arguments: ARGUMENT_LIST,
    ) -> Optional[Tuple]:
        description = api_full[3]
        res = self._target_api[description](**arguments)
        return res

    def init_demo(
        self,
        walk_path: str = DEFAULT_WALK_PATH,
    ):
        self.spot_client.upload_graph(walk_path)
        self.spot_client.set_localization_fiducial()

    def publish_debug_data(self, string_type: str, data):
        self.pub_debug_string.publish(
            String(
                data=yaml.dump(
                    {
                        "string_type": string_type,
                        "data": data,
                    }
                )
            )
        )

    def call_api(
        self,
        intension: str,
    ) -> Optional[Tuple]:
        rospy.loginfo(f"Calling api from intension: {intension}")
        api_full_list = get_api_list(self.sdp_interface)
        spot_api_full_list = self.get_spot_api_list()
        api_full_list += spot_api_full_list
        self.publish_debug_data(
            "api_full_list",
            convert_api_type_list_to_string_ready(api_full_list),
        )
        api_short_list = [
            (api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list
        ]
        similarity_list = []
        target_api_list_short_with_similarity = []
        for (
            description_api,
            api_arguments,
            api_response,
        ) in api_short_list:
            similarity = self.discovery._calc_semantic_similarity(
                intension,
                {},
                [],
                description_api,
                api_arguments,
                api_response,
            )
            similarity_list.append(similarity)
            if similarity > 0.45:
                selected_api = (
                    description_api,
                    api_arguments,
                    api_response,
                )
                target_api_list_short_with_similarity.append((similarity, selected_api))
        self.publish_debug_data(
            "api_similarity_list",
            [
                {
                    "api": convert_api_type_to_string_ready(api_full),
                    "intension": intension,
                    "arguments": {},
                    "response_names_and_types": [],
                    "similarity": similarity,
                }
                for api_full, similarity in zip(api_full_list, similarity_list)
            ],
        )
        target_api_list_short = [
            target_api_short
            for similarity, target_api_short in target_api_list_short_with_similarity
        ]
        target_api_list_full = [
            api_full_list[api_short_list.index(target_api_short)]
            for target_api_short in target_api_list_short
        ]
        # rospy.loginfo(f"target_api: {target_api_list_short}")
        self.publish_debug_data(
            "target_api_selection",
            [
                {
                    "api": convert_api_type_to_string_ready(target_api_full),
                    "intension": intension,
                    "arguments": {},
                    "response_names_and_types": [],
                    "similarity": target_api_short_with_similarity[0],
                }
                for target_api_full, target_api_short_with_similarity in zip(
                    target_api_list_full,
                    target_api_list_short_with_similarity,
                )
            ],
        )
        if (target_api_list_full) == 0:
            rospy.logerr("No suitable API found")
            return None
        if target_api_list_full[0][0] == (0, 0, 0, 0, 0, 0):
            rospy.loginfo("Calling Robot API")
            target_api_full = target_api_list_full[0]
            target_api_short = target_api_list_short[0]
            target_api_args = self.completion.generate_arguments_for_api(
                intension,
                {},
                [],
                target_api_short[0],
                target_api_short[1],
                target_api_short[2],
            )
            self.publish_debug_data(
                "api_call",
                {
                    "api": convert_api_type_to_string_ready(target_api_full),
                    "arguments": target_api_args,
                },
            )
            rospy.loginfo(
                f"api_calling: api: {target_api_short}, args: {target_api_args}"
            )
            res = self.call_spot_api(target_api_full, target_api_args)
            self.publish_debug_data(
                "api_response",
                {
                    "api": convert_api_type_to_string_ready(target_api_full),
                    "response": res,
                },
            )
            return res
        else:  # For device
            rospy.loginfo(
                "Calling Device API because top API is not Robot API: %s(%f)",
                target_api_list_full[0],
                target_api_list_short_with_similarity[0][0],
            )
            # Get closest target api
            target_api_full = None
            distance_to_base = float("inf")
            device_interfaces = self.sdp_interface.device_interfaces
            self.publish_debug_data(
                "device_distances",
                [
                    {
                        "address": addr,
                        "device_name": dev_if["device_name"],
                        "distance": (
                            None
                            if dev_if["distance"] is None
                            else (
                                None
                                if rospy.Time.now() - dev_if["distance_stamp"]
                                > rospy.Duration(10.0)
                                else dev_if["distance"]
                            )
                        ),
                    }
                    for addr, dev_if in device_interfaces.items()
                ],
            )
            # rospy.loginfo("device_interfaces: %s", device_interfaces)
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
                    rospy.logerr(
                        f"Distance for {device_name} is too old: {distance_stamp}"
                    )
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
            rospy.loginfo(
                f"api_calling: api: {target_api_short}, args: {target_api_args}"
            )
            self.publish_debug_data(
                "api_call",
                {
                    "api": convert_api_type_to_string_ready(target_api_full),
                    "arguments": target_api_args,
                },
            )
            res = call_api(self.sdp_interface, target_api_full, target_api_args)
            time.sleep(5.0)
            self.publish_debug_data(
                "api_response",
                {
                    "api": convert_api_type_to_string_ready(target_api_full),
                    "response": res,
                },
            )
            return res

    def run_demo(
        self,
        room_name: str = "73A4",
    ):

        self.call_api("Move to the front of room {}".format(room_name))
        self.call_api("Move into the room {}".format(room_name))
        self.call_api("Speak greeting")
        self.call_api("Move out of the room {}".format(room_name))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=["73A4", "73B1", "73B2"], default="73A4")
    parser.add_argument("--init", action="store_true")
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    time.sleep(5.0)
    if args.init:
        demo.init_demo(args.target)
    input("Press Enter to start the demo")
    demo.run_demo(args.target)
