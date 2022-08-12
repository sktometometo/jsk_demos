#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch-wandering")
(fetch-init)

(setq *max-x-range* (ros::get-param "~max_x_range" 1.0))
(setq *min-x-range* (ros::get-param "~min_x_range" 0.0))
(setq *max-y-range* (ros::get-param "~max_y_range" 1.0))
(setq *min-y-range* (ros::get-param "~min_y_range" -1.0))
(setq *max-theta-range* (ros::get-param "~max_theta_range" 3.14))
(setq *min-theta-range* (ros::get-param "~min_theta_range" -3.14))

(setq *ros-lock* (instance ros-lock :init "navigation"))
(send *ros-lock* :wait-for-server)

(ros::rate 0.1)
(while (ros:ok)
  (ros::sleep)
  (ros::spin-once)
  (send *ros-lock* :acquire)
  (send *go-pos-no-wait*
        (- (random (- *max-x-range* *min-x-range*)) *min-x-range*)
        (- (random (- *max-y-range* *min-y-range*)) *min-y-range*)
        (rad2deg (- (random (- *max-theta-range* *min-theta-range*)) *min-theta-range*))
        )
  (send *ros-lock* :release)
  )
