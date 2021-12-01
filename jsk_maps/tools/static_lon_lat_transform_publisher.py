#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import staticmap


# Copied from https://github.com/komoot/staticmap/blob/master/staticmap/staticmap.py
# original code is distributed with Apache 2.0
def lon_to_x(lon, zoom):
    """
    transform longitude to tile number
    :type lon: float
    :type zoom: int
    :rtype: float
    """
    if not (-180 <= lon <= 180):
        lon = (lon + 180) % 360 - 180

    return ((lon + 180.) / 360) * pow(2, zoom)


# Copied from https://github.com/komoot/staticmap/blob/master/staticmap/staticmap.py
# original code is distributed with Apache 2.0
def lat_to_y(lat, zoom):
    """
    transform latitude to tile number
    :type lat: float
    :type zoom: int
    :rtype: float
    """
    if not (-90 <= lat <= 90):
        lat = (lat + 90) % 180 - 90

    return (1 - math.log(math.tan(lat * math.pi / 180) + 1 / math.cos(lat * math.pi / 180)) / math.pi) / 2 * pow(2, zoom)


def calc_transform_from_lon_lat(
                    from_longitude,
                    from_latitude,
                    to_longitude,
                    to_latitude,
                    zoom=18,
                    tile_size=256,
                    resolution=0.484864926633 ):
    from_x_meter = lon_to_x(from_longitude, zoom) * tile_size * resolution
    from_y_meter = lat_to_y(-from_latitude, zoom) * tile_size * resolution
    to_x_meter = lon_to_x(to_longitude, zoom) * tile_size * resolution
    to_y_meter = lat_to_y(-to_latitude, zoom) * tile_size * resolution
    diff_x_meter = to_x_meter - from_x_meter
    diff_y_meter = to_y_meter - from_y_meter
    return diff_x_meter, diff_y_meter


class StaticLonLatPublisher:

    def __init__(self):

        self.from_frame_id = rospy.get_param('~from_frame_id')
        self.from_latitude = rospy.get_param('~from_latitude')
        self.from_longitude = rospy.get_param('~from_longitude')

        self.to_frame_id = rospy.get_param('~to_frame_id')
        self.to_latitude = rospy.get_param('~to_latitude')
        self.to_longitude = rospy.get_param('~to_longitude')

        self.tf_br = tf2_ros.StaticTransformBroadcaster()

        diff_x, diff_y = calc_transform_from_lon_lat(
                            self.from_longitude,
                            self.from_latitude,
                            self.to_longitude,
                            self.to_latitude )

        transform = TransformStamped()
        transform.header.frame_id = self.from_frame_id
        transform.child_frame_id = self.to_frame_id
        transform.transform.translation.x = diff_x
        transform.transform.translation.y = diff_y
        transform.transform.rotation.w = 1.0

        self.tf_br.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('static_lon_lat_publisher', anonymous=True)
    node = StaticLonLatPublisher()
    rospy.spin()

