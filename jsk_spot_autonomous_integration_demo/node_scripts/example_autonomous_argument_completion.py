#!/usr/bin/env python

import argparse
from typing import Any, Callable, Dict, List, Optional, Tuple

from autonomous_integration import ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    parser.add_argument("--example-number", type=int, default=0)
    args = parser.parse_args()

    completion = ArgumentCompletion(api_key=args.api_key)

    if args.example_number == 0:
        example_api: Tuple[
            str,
            ARGUMENT_NAMES_AND_TYPES,
            RESPONSE_NAMES_AND_TYPES,
        ] = (
            "Set the light status",
            [("turn_on", "Bool")],
            [("success", "Bool")],
        )
        intension_description = "Turn on the light"
    elif args.example_number == 1:
        example_api = (
            "SDP Lock 73B2: Key control",
            [("arg0", "string")],
            [],
        )
        intension_description = "Close door lock"
    else:
        raise ValueError(f"Unknown example number: {args.example_number}")

    args = completion.generate_arguments_for_api(
        intension_description,
        {},
        [],
        example_api[0],
        example_api[1],
        example_api[2],
    )

    print(args)
