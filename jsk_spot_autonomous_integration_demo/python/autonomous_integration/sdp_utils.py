from typing import List, Tuple
from urllib import response

from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)

from . import ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES


def get_api_list(
    interface: UWBSDPInterface,
) -> List[Tuple[str, str, str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]]:
    """
        Convert the list of device information to the list of API information.

    Returns:
        List[Tuple[str, str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]]: List of API information.
            Device name, pub or sub, description, argument names and types, response names and types.

    """
    api_list: List[
        Tuple[str, str, str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]
    ] = []
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
                    (device_name, "pub", interface[0], arguments_name_and_types, [])
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
                    (device_name, "sub", interface[0], [], response_name_and_types)
                )
    return api_list
