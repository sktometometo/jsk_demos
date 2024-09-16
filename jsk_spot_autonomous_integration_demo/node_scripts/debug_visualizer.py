#!/usr/bin/env python

import threading
from typing import Any, Dict, List, Optional, Tuple

import rospy
import yaml
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import String


class DebugVisualizer:

    def __init__(self):

        self._sub = rospy.Subscriber("/debug_text", String, self._callback)
        self._pub_api_list = rospy.Publisher(
            "/debug_text_overlay/api_list", OverlayText, queue_size=1
        )
        self._pub_target_api = rospy.Publisher(
            "/debug_text_overlay/target_api", OverlayText, queue_size=1
        )
        self._pub_api_call = rospy.Publisher(
            "/debug_text_overlay/api_call", OverlayText, queue_size=1
        )

        self._api_list: List[Tuple[str, str, List[str]]] = []
        self._api_list_stamp = rospy.Time.now()
        self._api_list_lock = threading.Lock()
        self._target_api: List[Tuple[str, str]] = []
        self._target_api_intension = ""
        self._target_api_stamp = rospy.Time.now()
        self._target_api_lock = threading.Lock()
        self._api_call: Optional[Tuple[str, str, Dict[str, Any]]] = None
        self._api_call_stamp = rospy.Time.now()
        self._api_call_lock = threading.Lock()

    def _callback(self, msg):

        data = yaml.load(msg.data)
        if data["string_type"] == "api_full_list":
            api_list = data["data"]
            with self._api_list_lock:
                self._api_list = [
                    (
                        api["device_name"],
                        api["description"],
                        [
                            name
                            for name, type in api["argument_names_and_types"].items()
                        ],
                    )
                    for api in api_list
                ]
                self._api_list_stamp = rospy.Time.now()
        elif data["string_type"] == "target_api_selection":
            target_api_list = data["data"]["target_apis"]
            intension = data["data"]["intension"]
            with self._target_api_lock:
                self._target_api = [
                    (api["device_name"], api["description"]) for api in target_api_list
                ]
                self._target_api_intension = intension
                self._target_api_stamp = rospy.Time.now()
        elif data["string_type"] == "api_call":
            api = data["data"]
            api_device_name = api["device_name"]
            api_description = api["description"]
            arguments = data["arguments"]
            with self._api_call_lock:
                self._api_call = (api_device_name, api_description, arguments)
                self._api_call_stamp = rospy.Time.now()
        else:
            rospy.logwarn("Invalid string type")
            return

    def _publish_api_call(self):

        with self._api_call_lock:
            if self._api_call is None:
                return
            api_device_name, api_description, arguments = self._api_call
            text = OverlayText()
            text.text = f"API call: {api_device_name}\n{api_description}\n"
            for name, value in arguments.items():
                text.text += f"{name}: {value}\n"
            self._pub_api_call.publish(text)

    def _publish_api_list(self):

        with self._api_list_lock and self._target_api_lock:
            text = OverlayText()
            text.text = "API list\n"
            for device_name, description, argument_names_and_types in self._api_list:
                text.text += f"{device_name}: {description}\n"
                for name in argument_names_and_types:
                    text.text += f"  {name}\n"
            self._pub_api_list.publish(text)
        
    
