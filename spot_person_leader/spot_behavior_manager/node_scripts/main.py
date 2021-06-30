#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from spot_behavior_manager.support_behavior_graph import SupportBehaviorGraph

class Demo(object):

    def __init__(self):

        # navigation dictonary
        edges = rospy.get_param('~map/edges')
        nodes = rospy.get_param('~map/nodes')
        self._map = SupportBehaviorGraph(edges,nodes)
        self._current_node = rospy.get_param('~initial_node')
        self._pre_edge = None

        # parameter
        self._duration_visible_timeout = rospy.get_param('~duration_visible_timeout', 10.0)

        #
        self._spot_client = SpotRosClient();
        self._sound_client = SoundClient(
                                    blocking=False,
                                    sound_action='/robotsound_jp',
                                    sound_topic='/robotsound_jp'
                                    )

        # publisher
        self._pub_current_node = rospy.Publisher('~/current_node',String,queue_size=1)

        # reset service
        self._service_reset = rospy.Service(
                                    '~reset_current_node',
                                    ResetCurrentNode,
                                    self.handler_reset_current_node
                                    )

        #
        roslaunch.pmon._init_signal_handlers()

        # action server
        self._server_lead_person = actionlib.SimpleActionServer(
                                        '~lead_person',
                                        LeadPersonAction,
                                        execute_cb=self.handler_lead_person,
                                        auto_start=False
                                        )
        self._server_lead_person.start()

        rospy.loginfo('Initialized!')


    def handler_reset_current_node(self, req):

        rospy.loginfo('Lead Action started.')

        path = self._map.calcPath( self._current_node, goal.target_node )

        self._sound_client.say('目的地に向かいます')

        for edge in path:
            try:
                if self.navigate_edge(edge):
                    rospy.loginfo('transition with edge {} succeeded'.format(edge))
                    self._current_node = edge['to']
                    self._pre_edge = edge
                else:
                    rospy.logerr('transition with edge {} failed'.format(edge))
                    result = LeadPersonResult(success=False)
                    self._sound_client.say('目的地に到達できませんでした'.format(goal.target_node),
                               volume=1.0)
                    self._server_lead_person.set_aborted(result)
                    return
            except Exception as e:
                rospy.logerr('Got an error with edge {}: {}'.format(edge, e))
                self._sound_client.say('エラーが発生しました'.format(goal.target_node),
                               volume=1.0)
                return

        self._sound_client.say('目的地に到着しました.'.format(goal.target_node),
                               volume=1.0)

        result = LeadPersonResult(success=True)
        self._server_lead_person.set_succeeded(result)
