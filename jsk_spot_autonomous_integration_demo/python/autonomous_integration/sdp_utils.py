from typing import List, Tuple
from urllib import response

from smart_device_protocol.smart_device_protocol_interface import (
    DataFrame,
    UWBSDPInterface,
)
from . import (
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
)


def get_api_list(
    interface: UWBSDPInterface,
) -> List[Tuple[str, str, str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]]:
    """
        Convert the list of device information to the list of API information.

    Returns:
        List[Tuple[str, str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]]: List of API information.
            Device name, pub or sub, description, argument names and types, response names and types.

        In [5]: interface.device_interfaces
    Out[5]:
    {(36,
      10,
      196,
      248,
      180,
      124): {'broadcast_interfaces': [('Key status', '?'),
       ('UWB Station',
        'i')], 'device_name': 'SDP Lock 73B2', 'distance': 76.1500015258789, 'interfaces': [('Key control',
        's')], 'last_stamp': rospy.Time[1677762647464644670], 'uwb_id': 1},
     (120, 33, 132, 149, 125, 148): {'device_name': 'SDP_DEBUG_BOARD',
      'distance': None,
      'interfaces': [],
      'last_stamp': rospy.Time[1677762647395696878],
      'uwb_id': None},
     (120, 33, 132, 149, 126, 152): {'device_name': 'msg_board',
      'distance': None,
      'interfaces': [('Message Board message', 'siS')],
      'last_stamp': rospy.Time[1677762647411919116],
      'uwb_id': None},
     (120,
      33,
      132,
      168,
      0,
      196): {'broadcast_interfaces': [('Light status', '?'),
       ('UWB Station',
        'i')], 'device_name': 'SDP Switchbot', 'distance': 58.34000015258789, 'interfaces': [('Light control',
        '?')], 'last_stamp': rospy.Time[1677762647317466974], 'uwb_id': 4}}
    """
    api_list = []
    for addr, dev_inf in interface.device_interfaces.items():
        device_name = dev_inf["device_name"]
        if "interfaces" in dev_inf:
            for interface in dev_inf["interfaces"]:
                arguments_name_and_types = []
                for i, arg_type_char in enumerate(interface[1]):
                    if arg_type_char == "s" or arg_type_char == "S":
                        arguments_name_and_types.append((f"arg{i}", "string"))
                    elif arg_type_char == "i":
                        arguments_name_and_types.append((f"arg{i}","int"))
                    elif arg_type_char == "f":
                        arguments_name_and_types.append((f"arg{i}","float"))
                    elif arg_type_char == "?" or arg_type_char == "b":
                        arguments_name_and_types.append((f"arg{i}","bool"))
                    else:
                        raise ValueError(f"Unknown argument type: {arg_type_char}")
                api_list.append((device_name, "pub", interface[0], arguments_name_and_types, None))
        if "broadcast_interfaces" in dev_inf:
            for interface in dev_inf["broadcast_interfaces"]:
                response_name_and_types = []
                for i, res_type_char in enumerate(interface[1]):
                    if res_type_char == "s" or res_type_char == "S":
                        response_name_and_types.append((f"res{i}", "string"))
                    elif res_type_char == "i":
                        response_name_and_types.append((f"res{i}","int"))
                    elif res_type_char == "f":
                        response_name_and_types.append((f"res{i}","float"))
                    elif res_type_char == "?" or res_type_char == "b":
                        response_name_and_types.append((f"res{i}","bool"))
                    else:
                        raise ValueError(f"Unknown response type: {res_type_char}")
                api_list.append((device_name, "sub", interface[0], None, response_name_and_types))
    return api_list
