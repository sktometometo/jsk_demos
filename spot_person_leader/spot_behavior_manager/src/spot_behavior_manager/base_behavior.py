import rospy
import roslaunch



class BaseBehavior(object):

    def __init__(self, spot_client, sound_client, name ):

        self.spot_client = spot_client
        self.sound_client = sound_client
        self.name = name

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
