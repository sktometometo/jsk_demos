#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import copy
import sys
import math

import rospy
import actionlib
import tf2_ros

from ros_lock import ROSLock
from ros_lock import roslock_acquire

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseResult
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetPlanRequest
from geometry_msgs.msg import PoseStamped


def calc_distance_poses(pose1, pose2):

    return math.sqrt((pose1.pose.position.x - pose2.pose.position.x)**2 +
                     (pose1.pose.position.y - pose2.pose.position.y)**2 +
                     (pose1.pose.position.z - pose2.pose.position.z)**2)


class LockedMoveBaseServer(object):

    def __init__(self):

        self.planning_tolerance = rospy.get_param('~planning_tolerance', 0.1)
        self.replanning_threshold_distance = rospy.get_param('~replanning_threshold_distance', 2.0)
        self.max_trial_number = rospy.get_param('~max_trial_number', 3)

        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')

        self.minimum_traverse_distance = rospy.get_param(
            '~minimum_traverse_distance', 2.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ros_lock = ROSLock()

        try:
            rospy.wait_for_service('~make_plan', timeout=rospy.Duration(30))
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('Planning service unavailable.: {}'.format(e))
            sys.exit(1)
        self.planning_client = rospy.ServiceProxy('~make_plan', GetPlan)

        self.move_base_client = actionlib.SimpleActionClient(
            '~target_move_base', MoveBaseAction)
        self.move_base_server = actionlib.SimpleActionServer(
            '~move_base', MoveBaseAction, self.execute, False)
        self.action_goal_pub = rospy.Publisher(
            '~move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.simple_move_base = rospy.Subscriber(
            '~move_base_simple/goal', PoseStamped, self.callback)

        self.move_base_server.start()

        rospy.loginfo('Initialized')

    def callback(self, msg):

        action_goal = MoveBaseActionGoal()
        action_goal.header.stamp = rospy.Time.now()
        action_goal.goal.target_pose = msg
        self.action_goal_pub.publish(action_goal)

    def feedback_cb(self, feedback):

        self.move_base_server.publish_feedback(feedback)

    def get_current_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rospy.Time(),
                rospy.Duration(1)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Error: {}'.format(e))
            return None

        current_pose = PoseStamped()
        current_pose.header = transform.header
        current_pose.pose.position.x = transform.transform.translation.x
        current_pose.pose.position.y = transform.transform.translation.y
        current_pose.pose.position.z = transform.transform.translation.z
        current_pose.pose.orientation.x = transform.transform.rotation.x
        current_pose.pose.orientation.y = transform.transform.rotation.y
        current_pose.pose.orientation.z = transform.transform.rotation.z
        current_pose.pose.orientation.w = transform.transform.rotation.w

        return current_pose

    def plan_path_and_get_waypoints(self, start_pose, goal_pose, tolerance):

        req = GetPlanRequest()
        req.tolerance = tolerance
        req.start = start_pose
        req.goal = copy.deepcopy(goal_pose)

        try:
            plan = self.planning_client(req).plan
        except Exception as e:
            rospy.logerr('Planning failed.: {}'.format(e))
            return None, 'Planning failed.: {}'.format(e)

        if len(plan.poses) == 0:
            rospy.logerr('No valid plan found.')
            return None, 'No valid plan found.'

        # Add direction to each pose and make them as list
        list_goals = []
        for index in range(len(plan.poses)):
            pose = plan.poses[index]
            if index < len(plan.poses) - 1:
                next_pose = plan.poses[index + 1]
                direction = math.atan2(next_pose.pose.position.y - pose.pose.position.y,
                                       next_pose.pose.position.x - pose.pose.position.x)
                pose.pose.orientation.z = math.sin(direction / 2.0)
                pose.pose.orientation.w = math.cos(direction / 2.0)
            if len(list_goals) == 0:
                if calc_distance_poses(start_pose, pose) > self.minimum_traverse_distance:
                    list_goals.append(MoveBaseGoal(pose))
            else:
                if calc_distance_poses(list_goals[-1].target_pose, pose) > self.minimum_traverse_distance:
                    list_goals.append(MoveBaseGoal(pose))
        if len(list_goals) == 0:
            list_goals.append(MoveBaseGoal(goal_pose))
        else:
            list_goals[-1] = MoveBaseGoal(goal_pose)

        return list_goals, 'Success'

    def control_goals(self, list_goals):

        for goal in list_goals:
            rospy.loginfo('goal: {}'.format(goal))
            with roslock_acquire(self.ros_lock, 'base'):
                # Where replanning is required or not
                if calc_distance_poses(self.get_current_pose(), goal.goal_pose) > self.replanning_threshold_distance:
                    rospy.logwarn('Robot is away from pre goal')
                    return False, True, 'Robot is away from pre goal'
                self.move_base_client.send_goal(
                    goal,
                    feedback_cb=self.feedback_cb
                )
                rate = rospy.Rate(1)
                while not rospy.is_shutdown():
                    rate.sleep()
                    if self.move_base_client.wait_for_result(timeout=rospy.Duration(1)):
                        break
                    if self.move_base_server.is_preempt_requested():
                        rospy.logerr('Cancel requested.')
                        return False, False, 'Cancel requested.'
                current_pose = self.get_current_pose()
                if calc_distance_poses(current_pose, goal.target_pose) > self.planning_tolerance + 1.0:
                    rospy.logerr('Failed to reach waypoints')
                    return False, True, 'Failed to reach waypoints'
            if rospy.is_shutdown():
                return False, False, None

    def execute(self, goal):

        for trial_index in range(self.max_trial_number):

            list_goals, message = self.plan_path_and_get_waypoints(
                    self.get_current_pose(),
                    goal.target_pose,
                    self.planning_tolerance
                    )

            if list_goals is None:
                rospy.logerr(message)
                result = MoveBaseResult()
                self.move_base_server.set_aborted(result)
                return

            success, retrial_flag, message = self.control_goals(list_goals)

            if retrial_flag:
                rospy.logwarn('Try to replan and execution')
                continue

            if success:
                rospy.loginfo('Goal Reached')
                result = MoveBaseResult()
                self.move_base_server.set_succeeded(result)
                return
            else:
                rospy.logerr(message)
                self.move_base_client.cancel_goal()
                result = self.move_base_client.get_result()
                self.move_base_server.set_aborted(result)
                return

        rospy.logerr('Robot cannot reach goal even after {} times of trials.'.format(self.max_trial_number))
        result = MoveBaseResult()
        self.move_base_server.set_aborted(result)


if __name__ == '__main__':

    rospy.init_node('locked_move_base_server')
    node = LockedMoveBaseServer()
    rospy.spin()
