#!/usr/bin/env python

import argparse
import time
from typing import Callable, List

import rospy
from autonomous_integration import (
    ARGUMENT_LIST,
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
)
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import call_from_intension
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)

LIGHT_STATUS_DESCRIPTION = "Get light status"
LIGHT_STATUS_ARGUMENT_NAMES_AND_TYPES: ARGUMENT_NAMES_AND_TYPES = []
LIGHT_STATUS_ARGUMENTS: ARGUMENT_LIST = {}
LIGHT_STATUS_RESPONSE_NAMES_AND_TYPES: RESPONSE_NAMES_AND_TYPES = [("lighted", "bool")]


def do_patrol_on_room(
    interface: UWBSDPInterface,
    discovery: ActiveAPIDiscovery,
    completion: ArgumentCompletion,
    func_enter: Callable,
    func_exit: Callable,
    func_get_people: Callable[[], List],
    func_notification: Callable[[str], None],
) -> DataFrame:

    func_enter()

    res = call_from_intension(
        interface,
        discovery,
        completion,
        "Get if the light is on",
        {},
        [],
    )
    turned_on_light = res["lighted"]

    if not turned_on_light:
        call_from_intension(
            interface,
            discovery,
            completion,
            "Turn on the light in the room.",
            {},
            [],
        )

    people = func_get_people()

    if len(people) > 0:
        func_notification("There are people in the room")
    else:
        call_from_intension(
            interface,
            discovery,
            completion,
            "Turn off the light in the room.",
            {},
            [],
        )

    func_exit()

    call_from_intension(
        interface,
        discovery,
        completion,
        "Lock the key of the room.",
        {},
        [],
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    parser.add_argument("--example-number", type=int, default=0)
    args = parser.parse_args()
