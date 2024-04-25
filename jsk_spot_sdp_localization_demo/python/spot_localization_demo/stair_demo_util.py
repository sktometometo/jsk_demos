from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional

import PyKDL
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class StairPointType(Enum):
    TOP_A = "TOP_A"
    TOP_B = "TOP_B"
    BOTTOM_A = "BOTTOM_A"
    BOTTOM_B = "BOTTOM_B"


@dataclass
class StairEntry:
    stair_name: str = ""
    frame_id: str = ""

    stair_top_a_device_name: Optional[str] = None
    stair_top_point_a: Optional[PyKDL.Vector] = None

    stair_top_b_device_name: Optional[str] = None
    stair_top_point_b: Optional[PyKDL.Vector] = None

    stair_bottom_a_device_name: Optional[str] = None
    stair_bottom_point_a: Optional[PyKDL.Vector] = None

    stair_bottom_b_device_name: Optional[str] = None
    stair_bottom_point_b: Optional[PyKDL.Vector] = None

    def is_complete(self) -> bool:
        return all(
            [
                self.stair_top_point_a is not None,
                self.stair_top_point_b is not None,
                self.stair_bottom_point_a is not None,
                self.stair_bottom_point_b is not None,
            ]
        )

    def is_in(self, point: PyKDL.Vector) -> bool:
        if not self.is_complete():
            return False

        return False
        


def get_markers_msg(
    stair_tables: Dict[str, StairEntry],
) -> MarkerArray:
    msg = MarkerArray()
    for stair_name, entry in stair_tables.items():
        if (
            entry.stair_top_point_a is not None
            and entry.stair_top_point_b is not None
            and entry.stair_bottom_point_a is not None
            and entry.stair_bottom_point_b is not None
        ):
            for point_name, point_type, point in [
                ("top_a", StairPointType.TOP_A, entry.stair_top_point_a),
                ("top_b", StairPointType.TOP_B, entry.stair_top_point_b),
                (
                    "bottom_a",
                    StairPointType.BOTTOM_A,
                    entry.stair_bottom_point_a,
                ),
                (
                    "bottom_b",
                    StairPointType.BOTTOM_B,
                    entry.stair_bottom_point_b,
                ),
            ]:
                marker = Marker()
                marker.header.frame_id = entry.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "stair"
                marker.id = hash(stair_name + point_name + "sphere") % 2147483648
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.a = 1.0
                msg.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = entry.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "stair"
            marker.id = hash(stair_name) % 2147483648
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.color.g = 1.0
            marker.color.a = 1.0
            for point_name, point_type, point in [
                ("top_a", StairPointType.TOP_A, entry.stair_top_point_a),
                ("top_b", StairPointType.TOP_B, entry.stair_top_point_b),
                (
                    "bottom_a",
                    StairPointType.BOTTOM_A,
                    entry.stair_bottom_point_a,
                ),
                (
                    "bottom_b",
                    StairPointType.BOTTOM_B,
                    entry.stair_bottom_point_b,
                ),
            ]:
                if point is None:
                    continue
                marker.points.append(Point(x=point.x(), y=point.y(), z=point.z()))
            msg.markers.append(marker)

    return msg
