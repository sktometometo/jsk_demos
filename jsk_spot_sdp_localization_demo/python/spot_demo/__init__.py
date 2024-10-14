import copy
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import PyKDL
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point
from jsk_recognition_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry
from smart_device_protocol.smart_device_protocol_interface import UWBSDPInterface
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uwb_localization.msg import SDPUWBDeviceArray


@dataclass
class TableEntry:
    device_name: str
    address: Optional[Tuple] = None
    header: Optional[Header] = None
    point: Optional[Point] = None


class SpotDemo:

    def __init__(self, people_bbox_topic_name="/spot_recognition/bbox_array"):

        self._reset_localize = rospy.ServiceProxy(
            "/device_localization_node/reset",
            Trigger,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.spot_client = SpotRosClient()
        self.sound_client = SoundClient()

        self.device_dict: Dict[str, TableEntry] = {}
        self.lock_device_dict = threading.Lock()

        self.sdp_interface = UWBSDPInterface()

        self._frame_odom_to_base: Optional[PyKDL.Frame] = None
        self._lock_frame_odom_to_base = threading.Lock()
        self._frame_id_odom: Optional[str] = None
        self._sub_odom = rospy.Subscriber(
            "/spot/odometry",
            Odometry,
            self._cb_odom,
        )

        self._odom_to_people: List[PyKDL.Frame] = []
        self._lock_people = threading.Lock()
        self._sub_people_bbox = rospy.Subscriber(
            people_bbox_topic_name,
            BoundingBoxArray,
            self._cb_people_bbox,
        )

        self._sub_uwb_device = rospy.Subscriber(
            "/sdpuwb_devices",
            SDPUWBDeviceArray,
            self._callback_uwb_device,
        )

    @property
    def frame_odom_to_base(self) -> PyKDL.Frame:
        rate = rospy.Rate(1)
        while True:
            with self._lock_frame_odom_to_base:
                if self._frame_odom_to_base is not None:
                    return copy.deepcopy(self._frame_odom_to_base)
            rate.sleep()

    @property
    def frame_id_odom(self) -> str:
        rate = rospy.Rate(1)
        while True:
            with self._lock_frame_odom_to_base:
                if self._frame_id_odom is not None:
                    return copy.deepcopy(self._frame_id_odom)
            rate.sleep()

    @property
    def odom_to_people(self) -> List[PyKDL.Frame]:
        return copy.deepcopy(self._odom_to_people)

    def _cb_odom(self, msg: Odometry):
        with self._lock_frame_odom_to_base:
            self._frame_id_odom = msg.header.frame_id
            self._frame_odom_to_base = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ),
                PyKDL.Vector(
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ),
            )

    def _look_up_transform(
        self, target_frame_id: str, base_frame_id: str = "base_link"
    ) -> Optional[PyKDL.Frame]:

        try:
            trans = self.tf_buffer.lookup_transform(
                base_frame_id,
                target_frame_id,
                rospy.Time(),
            )
            return tf2_geometry_msgs.transform_to_kdl(trans)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def _cb_people_bbox(self, msg: BoundingBoxArray):
        with self._lock_people:
            self._odom_to_people = [
                PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        msg_box.pose.orientation.x,
                        msg_box.pose.orientation.y,
                        msg_box.pose.orientation.z,
                        msg_box.pose.orientation.w,
                    ),
                    PyKDL.Vector(
                        msg_box.pose.position.x,
                        msg_box.pose.position.y,
                        msg_box.pose.position.z,
                    ),
                )
                for msg_box in msg.boxes
            ]

    def _callback_uwb_device(self, msg: SDPUWBDeviceArray):

        device_interfaces = copy.deepcopy(self.sdp_interface.device_interfaces)
        device_interfaces = {
            dev_if["device_name"]: dev_if["interfaces"]
            for addr, dev_if in device_interfaces.items()
        }
        for uwb_dev in msg.devices:
            assert uwb_dev.header.frame_id == self.frame_id_odom
            if uwb_dev.device_name in device_interfaces:
                with self.lock_device_dict:
                    if uwb_dev.device_name not in self.device_dict:
                        self.device_dict[uwb_dev.device_name] = TableEntry(
                            header=uwb_dev.header,
                            point=uwb_dev.point,
                            device_name=uwb_dev.device_name,
                        )
                    else:
                        self.device_dict[uwb_dev.device_name].header = uwb_dev.header
                        self.device_dict[uwb_dev.device_name].point = uwb_dev.point

        with self.lock_device_dict:
            for device_name, device_info in self.device_dict.items():
                if device_name not in [uwb_dev.device_name for uwb_dev in msg.devices]:
                    device_info.header = None
                    device_info.point = None
