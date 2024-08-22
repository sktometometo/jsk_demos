#!/usr/bin/env python

import argparse
from typing import Any, Callable, Dict, List, Optional, Tuple

from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration import (
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    args = parser.parse_args()

    discovery = ActiveAPIDiscovery(api_key=args.api_key)

    example_api_list: List[
        Tuple[
            str,
            ARGUMENT_NAMES_AND_TYPES,
            RESPONSE_NAMES_AND_TYPES,
        ]
    ] = [
        (
            "Get the thermostat status",
            [],
            [
                ("thermostat_id", "str"),
                ("temperature", "float"),
                ("humidity", "float"),
            ],
        ),
        (
            "Call a elevator",
            [("direction", "str")],
            [],
        ),
        (
            "Get the weather",
            [("location", "str")],
            [
                ("temperature", "float"),
                ("humidity", "float"),
            ],
        ),
        (
            "Get the light status",
            [],
            [("is_lighted", "bool")],
        ),
        (
            "Set the light status",
            [("turn_on", "bool")],
            [("success", "bool")],
        ),
        (
            "Get the door status",
            [],
            [("open", "bool")],
        ),
        (
            "Get the lock status",
            [],
            [("locked", "bool")],
        ),
        (
            "Set the lock status",
            [("locked", "bool")],
            [("success", "bool")],
        ),
    ]

    results = discovery.select_api(
        "Turn on the light",
        [],
        [],
        example_api_list,
    )

    print(results)
