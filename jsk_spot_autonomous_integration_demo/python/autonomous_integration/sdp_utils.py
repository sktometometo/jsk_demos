import time
from enum import Enum
from typing import Any, List, Optional, Tuple, Union

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


API_TYPE = Tuple[
    Tuple[int, int, int, int, int, int],
    str,
    SDPType,
    str,
    str,
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
]


def call_from_intension(
    interface: UWBSDPInterface,
    discovery: ActiveAPIDiscovery,
    completion: ArgumentCompletion,
    description_intension: str,
    arguments_intension: ARGUMENT_LIST,
    response_names_and_types_intension: RESPONSE_NAMES_AND_TYPES,
) -> Optional[Tuple]:
    arguments_names_and_types = []
    for arg_name, arg in arguments_intension.items():
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
    api_full_list = get_api_list(interface)
    target_api = discovery.select_api(
        description_intension,
        arguments_names_and_types,
        response_names_and_types_intension,
        [(api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list],
    )
    if target_api is None:
        return None
    target_api_full = api_full_list[
        [(api[1] + ": " + api[3], api[5], api[6]) for api in api_full_list].index(
            target_api
        )
    ]
    target_api_args = completion.generate_arguments_for_api(
        description_intension,
        arguments_intension,
        response_names_and_types_intension,
        target_api[0],
        target_api[1],
        target_api[2],
    )
    return call_api(interface, target_api_full, target_api_args)


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
        return tuple()
    else:
        ans = None

        def callback(
            address: Union[List[int], Tuple[int, int, int, int, int, int]],
            contents: List,
        ):
            if address == api[0]:
                # Update ans with contents
                nonlocal ans
                ans = tuple(contents)

        interface.register_interface_callback((api[3], api[4]), callback)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if ans is not None:
                interface.unregister_interface_callback(callback)
                return ans
        interface.unregister_interface_callback(callback)
        return ans


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
                    if arg_type_char == "s" or arg_type_char == "S":
                        arguments_name_and_types.append((f"arg{i}", "string"))
                    elif arg_type_char == "i":
                        arguments_name_and_types.append((f"arg{i}", "int"))
                    elif arg_type_char == "f":
                        arguments_name_and_types.append((f"arg{i}", "float"))
                    elif arg_type_char == "?" or arg_type_char == "b":
                        arguments_name_and_types.append((f"arg{i}", "bool"))
                    else:
                        raise ValueError(f"Unknown argument type: {arg_type_char}")
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
