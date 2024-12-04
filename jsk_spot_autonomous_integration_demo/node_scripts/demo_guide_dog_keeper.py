#!/usr/bin/env python


import rospy
from ros_lock import ROSLock
from spot_demo import SpotDemo


class GuideDogDistanceKeeper(SpotDemo):

    def __init__(self, distance_threshold: float = 5.0, timeout: float = 5.0):
        super().__init__(people_bbox_topic_name="/spot/tracked_world_objects")
        self._distance_threshold = distance_threshold
        self._timeout = timeout
        self._mobility_lock = ROSLock("mobility")

    def spin(self, hz=1.0):
        last_found_stamp = rospy.Time.now()
        acquired = False
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            rate.sleep()
            frame_odom_to_base = self.frame_odom_to_base
            frames_odom_to_people = self.odom_to_people
            distance = None
            for frame_odom_to_people in frames_odom_to_people:
                dist = (frame_odom_to_people.p - frame_odom_to_base.p).Norm()
                if distance is None or dist < distance:
                    distance = dist
            if distance is None:
                rospy.loginfo("Person not found")
            elif distance < self._distance_threshold:
                rospy.loginfo(
                    "Person founded in {} m (threshold {})".format(
                        distance, self._distance_threshold
                    )
                )
                last_found_stamp = rospy.Time.now()
            else:
                rospy.loginfo(
                    "Person founded in {} m (threshold {}) but out of range".format(
                        distance, self._distance_threshold
                    )
                )
            if rospy.Time.now() - last_found_stamp > rospy.Duration(self._timeout):
                if not acquired:
                    acquired = True
                    self._mobility_lock.acquire()
                    rospy.loginfo("acquire")
            else:
                if acquired:
                    acquired = False
                    self._mobility_lock.release()
                    rospy.loginfo("release")


if __name__ == "__main__":
    rospy.init_node("demo_distance_keeper")
    threshold=rospy.get_param('~threshold', 3.0)
    timeout=rospy.get_param('~timeout', 0.5)
    hz=rospy.get_param('~hz', 10.0)
    node = GuideDogDistanceKeeper(distance_threshold=threshold, timeout=timeout)
    node.spin(hz=hz)
