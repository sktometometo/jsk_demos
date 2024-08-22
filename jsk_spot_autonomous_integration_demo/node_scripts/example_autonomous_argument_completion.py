#!/usr/bin/env python

import argparse
from typing import Any, Callable, Dict, List, Optional, Tuple

from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration import (
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    args = parser.parse_args()

    completion = ArgumentCompletion(api_key=args.api_key)

    example_api: Tuple[
        str,
        ARGUMENT_NAMES_AND_TYPES,
        RESPONSE_NAMES_AND_TYPES,
    ] = (
        "Set the light status",
        [("turn_on", "Bool")],
        [("success", "Bool")],
    )

    args = completion.generate_arguments_for_api(
        "Turn on the light",
        {},
        [],
        example_api[0],
        example_api[1],
        example_api[2],
    )

    print(args)
