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
from smart_device_protocol.smart_device_protocol_interface import \
    UWBSDPInterface
from sound_play.libsoundplay import SoundClient
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Header, String

from jsk_spot_autonomous_integration_demo.python.autonomous_integration.sdp_utils import \
    call_api


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
            "/speech_recognition_server/result",
            SpeechRecognitionCandidates,
            self.cb_stt,
        )

    def cb_stt(self, msg: SpeechRecognitionCandidates):
        message = msg.transcript[0]
        rospy.loginfo(f"STT: {message}")
        ans = self.call_device(message)
        rospy.loginfo(f"Answer: {ans}")

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

        target_api_list_short = self.discovery.select_api(
            intension, {}, [], api_short_list
        )
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
        for target_api_full_candidate in target_api_list_full:
            device_name = target_api_full_candidate[1]
            # Get dev_info for device_name from device_interfaces
            dev_info = None
            for addr, dev_if in device_interfaces.items():
                if dev_if["device_name"] == device_name:
                    dev_info = dev_if
                    break
            if dev_info is None:
                continue
            if "distance" not in dev_info or dev_info["distance"] is None:
                continue
            distance = dev_info["distance"]
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
