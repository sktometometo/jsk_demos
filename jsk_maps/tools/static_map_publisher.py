#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid

from staticmap import StaticMap

class StaticMapPublisher:

    def __init__(self):

        self.center_longitude = None
        self.center_latitude = None

        map_width = rospy.get_param('~map_width',1000)
        map_height = rospy.get_param('~map_height',1000)
        zoom_level = rospy.get_param('~zoom_level', 
        map_resolution = rospy.get_param('~map_resolution',1)

        self.static_map = StaticMap( map_width,
                                     map_height,

        self.pub_map_meta_data

    def render_map(self):

        
