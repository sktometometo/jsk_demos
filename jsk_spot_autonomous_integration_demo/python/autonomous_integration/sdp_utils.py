import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import rospy
from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)

from . import ARGUMENT_LIST, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES
from .active_api_discovery import ActiveAPIDiscovery
from .autonomous_argument_completion import ArgumentCompletion


class SDPType(Enum):
    PUB = "pub"
    SUB = "sub"
    SPOT = "spot"


API_TYPE = Tuple[
    Tuple[int, int, int, int, int, int],  # Address, [0, 0, 0, 0, 0, 0] for spot
    str,  # Device name, e.g. "Robot API" for Spot
    SDPType,  # Type
    str,  # Description
    str,  # Serialization format, not used for SPOT
    ARGUMENT_NAMES_AND_TYPES,  # Argument names and types
    RESPONSE_NAMES_AND_TYPES,  # Response names and types
]

API_TYPE_SHORT = Tuple[str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]


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


def get_type_string_from_arg(arg: Any) -> str:
    if isinstance(arg, str):
        return "string"
    elif isinstance(arg, int):
        return "int"
    elif isinstance(arg, float):
        return "float"
    elif isinstance(arg, bool):
        return "bool"
    else:
        raise ValueError(f"Unknown argument type: {arg}")


def get_type_string_from_type(arg_type: Any) -> str:
    if arg_type == str:
        return "string"
    elif arg_type == int:
        return "int"
    elif arg_type == float:
        return "float"
    elif arg_type == bool:
        return "bool"
    else:
        raise ValueError(f"Unknown argument type: {arg_type}")


def get_arguments_list_from_function(func: Callable) -> ARGUMENT_NAMES_AND_TYPES:
    return [
        (arg_name, get_type_string_from_type(arg_type))
        for arg_name, arg_type in func.__annotations__.items()
        if arg_name != "return"
    ]


def get_response_list_from_function(func: Callable) -> RESPONSE_NAMES_AND_TYPES:
    return_type = func.__annotations__["return"]
    if return_type is None:
        return []
    elif isinstance(return_type, tuple):
        return [
            (f"res{i}", get_type_string_from_type(arg_type))
            for i, arg_type in enumerate(return_type)
        ]
    else:
        return [("res0", get_type_string_from_type(return_type))]


def convert_args_to_argnames_and_types(args: ARGUMENT_LIST) -> ARGUMENT_NAMES_AND_TYPES:
    arguments_names_and_types = []
    for arg_name, arg in args.items():
        if isinstance(arg, str):
            arguments_names_and_types.append((arg_name, "string"))
        elif isinstance(arg, int):
            arguments_names_and_types.append((arg_name, "int"))
        elif isinstance(arg, float):
            arguments_names_and_types.append((arg_name, "float"))
        elif isinstance(arg, bool):
            arguments_names_and_types.append((arg_name, "bool"))
        else:
            raise ValueError(f"Unknown argument type: {arg}")
    return arguments_names_and_types


def call_api(
    interface: UWBSDPInterface,
    api: API_TYPE,
    arguments: ARGUMENT_LIST,
    timeout: float = 5.0,
) -> Optional[Tuple]:
    if len(arguments) != len(api[5]):
        raise ValueError(
            f"Number of arguments do not match. Expected {len(api[5])}, got {len(arguments)}"
        )
    content = [arguments[arg[0]] for arg in api[5]]
    if api[2] == SDPType.PUB:
        interface.send(
            api[0],
            DataFrame(
                packet_description=api[3],
                serialization_format=api[4],
                content=content,
            ),
        )
        time.sleep(5.0)
        return tuple()
    else:
        ans = None

        def callback(
            address: Union[List[int], Tuple[int, int, int, int, int, int]],
            data_frame: DataFrame,
        ):
            if address == api[0]:
                # Update ans with contents
                nonlocal ans
                ans = tuple(data_frame.content)

        interface.register_interface_callback((api[3], api[4]), callback)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if ans is not None:
                interface.unregister_interface_callback((api[3], api[4]))
                return ans
        interface.unregister_interface_callback((api[3], api[4]))
        return ans


def convert_format_char_to_type_string(format_char: str) -> str:
    if format_char == "s" or format_char == "S":
        return "string"
    elif format_char == "i":
        return "int"
    elif format_char == "f":
        return "float"
    elif format_char == "?" or format_char == "b":
        return "bool"
    else:
        raise ValueError(f"Unknown format char: {format_char}")


def convert_type_string_to_format_char(type_string: str) -> str:
    if type_string == "string":
        return "S"
    elif type_string == "int":
        return "i"
    elif type_string == "float":
        return "f"
    elif type_string == "bool":
        return "b"
    else:
        raise ValueError(f"Unknown type string: {type_string}")


def get_api_list(
    interface: UWBSDPInterface,
) -> List[API_TYPE]:
    """
        Convert the list of device information to the list of API information.

    Returns:
        List[Tuple[str, str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]]: List of API information.
            Device name, pub or sub, description, argument names and types, response names and types.

    """
    api_list: List[API_TYPE] = []
    for addr, dev_inf in interface.device_interfaces.items():
        device_name = dev_inf["device_name"]
        if "interfaces" in dev_inf:
            for interface in dev_inf["interfaces"]:
                arguments_name_and_types = []
                for i, arg_type_char in enumerate(interface[1]):
                    arguments_name_and_types.append(
                        (f"arg{i}", convert_format_char_to_type_string(arg_type_char))
                    )
                api_list.append(
                    (
                        addr,
                        device_name,
                        SDPType.PUB,
                        interface[0],
                        interface[1],
                        arguments_name_and_types,
                        [],
                    )
                )
        if "broadcast_interfaces" in dev_inf:
            for interface in dev_inf["broadcast_interfaces"]:
                response_name_and_types = []
                for i, res_type_char in enumerate(interface[1]):
                    if res_type_char == "s" or res_type_char == "S":
                        response_name_and_types.append((f"res{i}", "string"))
                    elif res_type_char == "i":
                        response_name_and_types.append((f"res{i}", "int"))
                    elif res_type_char == "f":
                        response_name_and_types.append((f"res{i}", "float"))
                    elif res_type_char == "?" or res_type_char == "b":
                        response_name_and_types.append((f"res{i}", "bool"))
                    else:
                        raise ValueError(f"Unknown response type: {res_type_char}")
                api_list.append(
                    (
                        addr,
                        device_name,
                        SDPType.SUB,
                        interface[0],
                        interface[1],
                        [],
                        response_name_and_types,
                    )
                )
    return api_list
