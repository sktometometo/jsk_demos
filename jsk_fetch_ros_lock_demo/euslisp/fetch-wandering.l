#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(require "package://ros_lock/euslisp/ros_lock.l")
(require "package://jsk_fetch_ros_lock_demo/euslisp/utils.l")

(ros::roseus "fetch_wandering")
(jsk-fetch-ros-lock-init :move-base-action-name "move_base_no_recovery_with_lock")

(ros::rate 1)
(while (ros::ok)
    (ros::sleep)
    (go-to-spot "/eng2/7f/room73A3-front")
    (ros::sleep)
    (go-to-spot "/eng2/7f/jsk-section-entrance")
    )
