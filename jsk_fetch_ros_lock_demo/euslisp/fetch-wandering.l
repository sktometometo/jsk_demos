#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "fetch-wandering")
(fetch-init)

(setq *max-x-range* (ros::get-param "~max_x_range" 0.5))
(setq *min-x-range* (ros::get-param "~min_x_range" 0.0))
(setq *max-y-range* (ros::get-param "~max_y_range" 0.5))
(setq *min-y-range* (ros::get-param "~min_y_range" -0.5))
(setq *max-theta-range* (ros::get-param "~max_theta_range" 3.14))
(setq *min-theta-range* (ros::get-param "~min_theta_range" -3.14))

(setq *use-ros-lock* (ros::get-param "~use_ros_lock" t))

(if *use-ros-lock*
  (progn
    (load "package://ros_lock/euslisp/ros_lock.l")
    (setq *ros-lock* (instance ros-lock :init "navigation"))
    (send *ros-lock* :wait-for-server)
    ))

(while (ros::ok)
  (ros::spin-once)
  (if *use-ros-lock*
      (send *ros-lock* :acquire)
      )
  (send *ri* :go-pos
        (- (random (- *max-x-range* *min-x-range*)) *min-x-range*)
        (- (random (- *max-y-range* *min-y-range*)) *min-y-range*)
        (rad2deg (- (random (- *max-theta-range* *min-theta-range*)) *min-theta-range*))
        )
  (if *use-ros-lock*
      (send *ros-lock* :release)
      )
  )
