#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import random
import sys
# ROS Libraries
import actionlib
import rospy
from spot_ros_client.libspotros import SpotRosClient
# ROS Messsages
from jsk_spot_delivery_demo.msg import PickupPackageAction
from jsk_spot_delivery_demo.msg import PickupPackageGoal


if __name__ == '__main__':

    rospy.init_node('random_delivery_demo')

    node_list = rospy.get_param('~node_list', [])

    return_after_demo = rospy.get_param('~return_after_demo', True)
    home_id = rospy.get_param('~home_id', 'eng2_73B2')

    timeout = rospy.get_param('~timeout', 120.0)
    execute_after_pickup = rospy.get_param('~execute_after_pickup', True)

    client = SpotRosClient()

    ac = actionlib.SimpleActionClient(
            '/delivery_action_server/pickup_package',
            PickupPackageAction
            )
    if not ac.wait_for_server(rospy.Duration(20)):
        rospy.logerr('Server is not available')
        sys.exit(1)

    goal = PickupPackageGoal()
    goal.timeout = rospy.Duration(timeout)
    goal.execute_after_pickup = execute_after_pickup

    result = ac.send_goal_and_wait(goal)
    rospy.loginfo('result: {}'.format(result))

    if return_after_demo:
        client.execute_behaviors(home_id)
        client.dock()
