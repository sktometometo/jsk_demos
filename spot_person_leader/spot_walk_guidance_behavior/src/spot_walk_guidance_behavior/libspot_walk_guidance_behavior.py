import rospy
import roslaunch
import rospack
from spot_base_behavior.libspot_base_behavior import BaseBehavior
from std_msgs.msg import Bool
import threading



class WalkGuidanceBehavior(BaseBehavior):

    def __init__(self, spot_client, sound_client, name ):

        super(WalkGuidanceBehavior,self).__init__(
                spot_client,
                sound_client,
                name
                )

    def __run_initial(self, start_node, end_node, edge, preedge ):

        # Start detection launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        detection_roslaunch_path = rospack.get_path('spot_walk_guidance_behavior') + '/launch/detection.launch'
        detection_roslaunch_cli_args = [detection_roslaunch_path]
        detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)

        self._tmp_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                        uuid,
                                        detection_roslaunch_file
                                        )

        # Start visibility subscriber
        self._tmp_state_visible = False
        self._tmp_starttime_visibility = rospy.Time.now()
        self._tmp_duration_visibility = rospy.Duration()

        self._tmp_subscriber_visible = rospy.Subscriber('/walk_detection_person_tracker/visible', Bool, self.callback_visible)

        rospy.logdebug('')

    def __run_finalize(self, start_node, end_node, edge, preedge ):

        self._tmp_roslaunch_parent.shutdown()
        self._tmp_subscriber_visible.unregister()

        del self._tmp_state_visible
        del self._tmp_starttime_visibility
        del self._tmp_duration_visibility
        del self._tmp_roslaunch_parent
        del self._tmp_subscriber_visible

    def __run_main(self, start_node, end_node, edge, preedge ):

        graph_name = edge['args']['graph']
        start_id = filter(
                    lambda x: x['graph'] == graph_name,
                    start_node['waypoints_on_graph']
                    )[0]['id']
        localization_method = filter(
                    lambda x: x['graph'] == graph_name,
                    start_node['waypoints_on_graph']
                    )[0]['localization_method']
        end_id = filter(
                    lambda x: x['graph'] == graph_name,
                    end_node['waypoints_on_graph']
                    )[0]['id']

        # graph uploading
        if pre_edge is not None and graph_name == pre_edge['args']['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            self.spot_client.upload_graph( graph_name )
            rospy.loginfo('graph {} uploaded.'.format(graph_name))
            # Localization
            if localization_method == 'fiducial':
                self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                self.spot_client.set_localization_waypoint(start_id)
            else:
                rospy.logerr('Unknown localization method')
                return False
            rospy.loginfo('robot is localized on the graph.')

        self.sound_client.say('ついてきてください',
                               volume=1.0,
                               blocking=True)

        success = False
        rate = rospy.Rate(10)
        self.spot_client.navigate_to( end_id, blocking=False)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                result = self.spot_client.get_navigate_to_result()
                success = result.success
                rospy.loginfo('result: {}'.format(result))
                break

            if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                flag_speech = False
                def notify_visibility():
                    self.sound_client.say(
                        '近くに人が見えません',
                        volume=1.0,
                        blocking=True
                        )
                    flag_speech = False
                speech_thread = threading.Thread(target=notify_visibility)
                speech_thread.start()
                while not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                    rate.sleep()
                    self.spot_client.pubCmdVel(0,0,0)
                    if not flag_speech:
                        flag_speech = True
                        speech_thread = threading.Thread(target=notify_visibility)
                        speech_thread.start()
                    if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(5.0):
                        self.spot_client.cancel_navigate_to()
                    if self._tmp_state_visible:
                        self.spot_client.navigate_to( end_id, blocking=False)

        # recovery
        if not success:
            self.sound_client.say(
                   '失敗したので元に戻ります',
                   volume=1.0,
                   blocking=True)
            self.spot_client.navigate_to( start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return success

    def callback_visible(self, msg):

        if self._tmp_state_visible != msg.data:
            self._tmp_starttime_visibility = rospy.Time.now()
            self._tmp_duration_visibility = rospy.Duration()
            self._tmp_state_visible = msg.data
        else:
            self._tmp_duration_visibility = rospy.Time.now() - self._tmp_starttime_visibility
