#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import actionlib
import rospy
import sys

from jsk_spot_delivery_demo.msg import PickupPackageAction
from jsk_spot_delivery_demo.msg import PickupPackageGoal


if __name__ == '__main__':

    rospy.init_node('pickup_package_client')

    timeout = rospy.get_param('~timeout', 120.0)
    execute_after_pickup = rospy.get_param('~execute_after_pickup', True)

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
