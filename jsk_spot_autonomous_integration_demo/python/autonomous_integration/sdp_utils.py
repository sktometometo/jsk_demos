import time
from enum import Enum
from typing import Any, List, Optional, Tuple, Union
from urllib import response

from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)

from . import ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES


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


def call_api(
    interface: UWBSDPInterface,
    api: API_TYPE,
    arguments: List[Any],
    timeout: float = 5.0,
) -> Optional[Tuple[Any]]:
    if api[2] == SDPType.PUB:
        interface.send(
            api[0],
            DataFrame(
                packet_description=api[3],
                serialization_format=api[4],
                content=arguments,
            ),
        )
        return None
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
        return None


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
