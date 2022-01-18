#!/usr/bin/env python
# -*- encoding: utf-8 -*-

"""this script is a demo that spot go to a specified spot"""

import rospy
from spot_ros_client.libspotros import SpotRosClient


def main():

    rospy.init_node('spot_go_to_spot')

    target_node_id = rospy.get_param('~target_node_id', 'eng2_7FElevator')
    num_retry = rospy.get_param('~num_retry', 3)

    client = SpotRosClient()
    client.auto_undock()

    rospy.loginfo('Start to go to {}'.format(target_node_id))
    for i in range(num_retry):
        result = client.execute_behaviors(target_node_id)
        if result.success:
            rospy.loginfo('Finished.')
            return
        else:
            rospy.logwarn('Failed. retrying...')

    rospy.logerr('Failed even after retrying.')


if __name__ == '__main__':
    main()
