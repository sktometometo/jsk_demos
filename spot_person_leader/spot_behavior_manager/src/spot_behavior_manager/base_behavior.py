import rospy
import roslaunch


def load_behavior_class(class_string):
    # TODO
    return SimpleBehavior

class BaseBehavior(object):

    def __init__(self, spot_client, sound_client):

        self.spot_client = spot_client
        self.sound_client = sound_client

    def __run_initial(self, start_node, end_node, edge, preedge ):

        rospy.logerr('Please override __run_main() method.')

    def __run_main(self, start_node, end_node, edge, preedge ):

        rospy.logerr('Please override __run_main() method.')
        return False

    def __run_finalize(self, start_node, end_node, edge, preedge ):

        rospy.logerr('Please override __run_finalize() method.')

    def run(self, start_node, end_node, edge, preedge ):

        try:
            self.__run_initial( start_node, end_node, edge, preedge )
            ret = self.__run__main( start_node, end_node, edge, preedge )
            self.__run_finalize( start_node, end_node, edge, preedge )
            return ret
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

class SimpleBehavior(BaseBehavior):

    def __run_initial(self, start_node, end_node, edge, preedge ):

        rospy.loginfo('__run_initial() called')

    def __run_main(self, start_node, end_node, edge, preedge ):

        rospy.loginfo('__run_main() called')
        return True

    def __run_finalize(self, start_node, end_node, edge, preedge ):

        rospy.loginfo('__run_finalize() called')
