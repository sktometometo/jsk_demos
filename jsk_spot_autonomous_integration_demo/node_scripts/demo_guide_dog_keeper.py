#!/usr/bin/env python


import rospy
from ros_lock import ROSLock
from spot_demo import SpotDemo


class GuideDogDistanceKeeper(SpotDemo):

    def __init__(self, distance_threshold: float = 5.0, timeout: float = 5.0):
        super().__init__()
        self._distance_threshold = distance_threshold
        self._timeout = timeout
        self._mobility_lock = ROSLock("mobility")

    def spin(self):
        last_found_stamp = rospy.Time.now()
        acquired = False
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            frame_odom_to_base = self.frame_odom_to_base
            frames_odom_to_people = self.odom_to_people
            distance = float("inf")
            for frame_odom_to_people in frames_odom_to_people:
                dist = (frame_odom_to_people.p - frame_odom_to_base.p).Norm()
                if dist < distance:
                    distance = dist
            if distance < self._distance_threshold:
                last_found_stamp = rospy.Time.now()
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
    node = GuideDogDistanceKeeper()
    node.spin()
