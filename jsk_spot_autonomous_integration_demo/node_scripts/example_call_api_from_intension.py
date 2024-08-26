#!/usr/bin/env python

import argparse
import time

import rospy
from autonomous_integration import ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import call_from_intension
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    parser.add_argument("--example-number", type=int, default=0)
    args = parser.parse_args()

    rospy.init_node("example_call_api_from_intension")

    completion = ArgumentCompletion(api_key=args.api_key)
    discovery = ActiveAPIDiscovery(api_key=args.api_key)
    interface = UWBSDPInterface()
    time.sleep(10.0)

    if args.example_number == 0:
        example_intension_description = "Turn on the light"
        example_intension_arguments = {}
        example_intension_response = []
    elif args.example_number == 1:
        example_intension_description = "Open the door lock"
        example_intension_arguments = {}
        example_intension_response = []
    else:
        raise ValueError(f"Unknown example number: {args.example_number}")

    res = call_from_intension(
        interface,
        discovery,
        completion,
        example_intension_description,
        example_intension_arguments,
        example_intension_response,
    )
    print(f"res: {res}")
