gimport json
import os
from curses import raw
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import yaml
from autonomous_integration import (ARGUMENT_NAMES_AND_TYPES,
                                    RESPONSE_NAMES_AND_TYPES,
                                    names_and_types_from_dict,
                                    names_and_types_to_dict)
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import \
    ArgumentCompletion
from autonomous_integration.sdp_utils import *
from openai import OpenAI

from .env import Environment


def call_device_openai(
    environment: Environment,
    intension: str,
    api_key: str,
) -> Optional[Tuple]:
    api_full_list = environment.get_api_list()

    client = OpenAI(api_key=api_key)

    TYPE_MAPPING = {
        "bool": "boolean",
        "int": "integer",
        "float": "number",
        "str": "string",
    }

    tools = []
    for api_full in api_full_list:
        properties = {
            name: {"type": TYPE_MAPPING.get(type_name, type_name)} for name, type_name in api_full[2]
        }
        required = list(properties.keys())
        tools.append(
            {
                "type": "function",
                "function": {
                    "name": api_full[0],
                    "description": api_full[1],
                    "parameters": {
                        "type": "object",
                        # "properties": {
                        #     name: {"type": type_name} for name, type_name in api_full[2]
                        # },
                        "properties": properties,
                        "required": required,
                        "additionalProperties": False,
                    },
                    "strict": True,
                },
            }
        )

    messages = [{"role": "user", "content": intension}]
    # rospy.loginfo(f"tools: {yaml.dump(tools)}")
    # rospy.loginfo(f"messages: {yaml.dump(messages)}")
    completion = client.chat.completions.create(
        model="gpt-4o",
        messages=messages,
        tools=tools,
    )
    # rospy.loginfo(f"completion: {completion}")
    if completion.choices[0].message.tool_calls is None:
        rospy.logwarn("No tool calls found in the completion.")
        return None
    try:
        target_function_name = completion.choices[0].message.tool_calls[0].function.name
        target_api_full = next(
            api_full for api_full in api_full_list if api_full[0] == target_function_name
        )
        raw_arguments = completion.choices[0].message.tool_calls[0].function.arguments
        arguments = yaml.safe_load(raw_arguments)
        target_function_args: ARGUMENT_LIST = {
            name: value
            for name, value in arguments.items()
        }

        return target_api_full, target_function_args
    except Exception as e:
        rospy.logerr(f"Error processing tool calls: {e}")
        return None
