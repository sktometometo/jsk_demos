#!/usr/bin/env python

import argparse
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import rospy
import yaml
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import \
    ArgumentCompletion
from autonomous_integration.sdp_utils import *
from autonomous_integration.sdp_utils import call_api
from smart_device_protocol.smart_device_protocol_interface import \
    UWBSDPInterface
from sound_play.libsoundplay import SoundClient
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Header, String


class Demo:

    def __init__(self):
        super().__init__()

        self.sdp_interface = UWBSDPInterface()
        self.discovery = ActiveAPIDiscovery()
        self.completion = ArgumentCompletion()

        self.pub_debug_string = rospy.Publisher(
            "/debug_string",
            String,
            queue_size=1,
        )

        self.sub_stt = rospy.Subscriber(
            "/speech_to_text",
            SpeechRecognitionCandidates,
            self.cb_stt,
        )

    def cb_stt(self, msg: SpeechRecognitionCandidates):
        message = msg.transcript[0]
        rospy.loginfo(f"STT: {message}")
        ans = self.call_device(message)
        rospy.loginfo(f"Answer: {ans}")

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

    def call_device(
        self,
        intension: str,
    ) -> Optional[Tuple]:
        api_full_list = get_api_list(self.sdp_interface)
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
            if similarity > 0.5:
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
        self.publish_debug_data(
            "api_call",
            {
                "api": convert_api_type_to_string_ready(target_api_full),
                "arguments": target_api_args,
            },
        )
        res = call_api(self.sdp_interface, target_api_full, target_api_args)
        self.publish_debug_data(
            "api_response",
            {
                "api": convert_api_type_to_string_ready(target_api_full),
                "response": res,
            },
        )
        time.sleep(5.0)
        return res


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    rospy.init_node("demo")
    demo = Demo()
    time.sleep(5.0)
    if args.debug:
        while not rospy.is_shutdown():
            message = input('Enter message ("exit" to exit): ')
            if message == "exit":
                break
            ans = demo.call_device(message)
            print(f"Answer: {ans}")
    else:
        rospy.spin()
