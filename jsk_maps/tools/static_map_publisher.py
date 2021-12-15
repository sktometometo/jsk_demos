#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from geometry_msgs.msg import TransformStamped

import numpy as np
import staticmap

from jsk_maps.srv import UpdateMapCenter
from jsk_maps.srv import UpdateMapCenterResponse
from jsk_maps import calc_transform_from_lon_lat
from jsk_maps import calc_meters_per_pixel


class StaticMapPublisher:

    def __init__(self):

        self.initial_latitude = rospy.get_param('~initial_latitude', 35.70751)
        self.initial_longitude = rospy.get_param(
            '~initial_longitude', 139.76060)
        self.zoom_level = int(rospy.get_param('~zoom_level', 18))
        self.map_size = int(rospy.get_param('~map_size', 1000))
        self.map_type = rospy.get_param(
            '~map_type', 'osm')  # 'osm' or 'gsi_jp'
        self.initial_frame = rospy.get_param(
            '~initial_frame', 'initial_frame')
        self.static_map_frame = rospy.get_param(
            '~static_map_frame', 'static_map_frame')

        self.center_latitude = self.initial_latitude
        self.center_longitude = self.initial_longitude
        self.transform_initial_to_static = TransformStamped()
        self.transform_initial_to_static.header.frame_id = self.initial_frame
        self.transform_initial_to_static.child_frame_id = self.static_map_frame
        self.transform_initial_to_static.transform.rotation.w = 1.0

        if self.map_type == 'osm':
            self.static_map = staticmap.StaticMap(
                self.map_size,
                self.map_size,
                url_template='https://tile.openstreetmap.org/{z}/{x}/{y}.png')
        elif self.map_type == 'gsi_jp':
            self.static_map = staticmap.StaticMap(
                self.map_size,
                self.map_size,
                url_template='https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png')

        self.msg_map_meta_data = MapMetaData()
        self.msg_occupancy_grid = OccupancyGrid()

        self.tf_br = tf2_ros.TransformBroadcaster()

        self.pub_map_meta_data = rospy.Publisher(
            '/map_meta_data', MapMetaData, queue_size=1, latch=True)
        self.pub_occupancy_grid = rospy.Publisher(
            '/map', OccupancyGrid, queue_size=1, latch=True)

        self.render_map()
        self.publish_map()

        self.srv_update_center = rospy.Service(
            '~update_center', UpdateMapCenter, self.handler_update_center)

        rospy.loginfo('Initialized')

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.calc_transform()
            self.publish_tf()
            rate.sleep()

    def handler_update_center(self, req):
        try:
            self.update_center(req.center_latitude, req.center_longitude)
            res = UpdateMapCenterResponse()
            res.success = True
            return res
        except Exception as e:
            res = UpdateMapCenterResponse()
            res.success = False
            res.message = '{}'.format(e)
            return res

    def update_center(self, latitude, longitude):
        self.center_latitude = latitude
        self.center_longitude = longitude
        self.render_map()
        self.publish_map()
        self.calc_transform()

    def calc_transform(self):
        # 地球を球体と仮定して、intial周りを平面近似して計算
        diff_x_meter, diff_y_meter = calc_transform_from_lon_lat(
            self.initial_longitude,
            self.initial_latitude,
            self.center_longitude,
            self.center_latitude,
            self.zoom_level
        )
        self.transform_initial_to_static.transform.translation.x = diff_x_meter
        self.transform_initial_to_static.transform.translation.y = diff_y_meter

    def render_map(self):
        map_resolution = calc_meters_per_pixel(
            self.center_latitude,
            self.zoom_level
        )
        rospy.logwarn('map resolution is {}'.format(map_resolution))
        try:
            image = self.static_map.render(
                zoom=self.zoom_level,
                center=(self.center_longitude, self.center_latitude)
            )
        except RuntimeError:
            rospy.logerr('Failed to download map images')
            self.msg_map_meta_data = MapMetaData()
            self.msg_occupancy_grid = OccupancyGrid()
            return

        self.msg_map_meta_data = MapMetaData()
        self.msg_map_meta_data.map_load_time = rospy.Time.now()
        self.msg_map_meta_data.resolution = map_resolution
        self.msg_map_meta_data.width = self.map_size
        self.msg_map_meta_data.height = self.map_size
        self.msg_map_meta_data.origin.position.x = -self.map_size * map_resolution / 2
        self.msg_map_meta_data.origin.position.y = self.map_size * map_resolution / 2
        self.msg_map_meta_data.origin.position.z = 0
        self.msg_map_meta_data.origin.orientation.x = 1.0
        self.msg_map_meta_data.origin.orientation.y = 0.0
        self.msg_map_meta_data.origin.orientation.z = 0.0
        self.msg_map_meta_data.origin.orientation.w = 0.0

        self.msg_occupancy_grid = OccupancyGrid()
        self.msg_occupancy_grid.header.frame_id = self.static_map_frame
        self.msg_occupancy_grid.header.stamp = rospy.Time.now()
        self.msg_occupancy_grid.info = self.msg_map_meta_data

        image_array = np.array(image.convert('L')) / 2
        self.msg_occupancy_grid.data = image_array.flatten().tolist()

    def publish_map(self):
        self.pub_map_meta_data.publish(self.msg_map_meta_data)
        self.pub_occupancy_grid.publish(self.msg_occupancy_grid)

    def publish_tf(self):
        self.tf_br.sendTransform(self.transform_initial_to_static)


if __name__ == '__main__':
    rospy.init_node('static_map_publisher')
    node = StaticMapPublisher()
    node.spin()
