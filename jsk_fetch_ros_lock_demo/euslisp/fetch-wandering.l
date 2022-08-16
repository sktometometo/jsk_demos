#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "fetch_wandering")
(setq *ri* (instance fetch-interface :init :move-base-action-name "move_base_no_recovery"))
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

(ros::rate 0.1)
(while (ros::ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (ros::sleep)
  (unix::sleep 1)
  (if *use-ros-lock*
      (send *ros-lock* :acquire)
      )
  (let ((target-x (+ (random (- *max-x-range* *min-x-range*)) *min-x-range*))
        (target-y (+ (random (- *max-y-range* *min-y-range*)) *min-y-range*))
        (target-theta (+ (random (- *max-theta-range* *min-theta-range*)) *min-theta-range*))
        )
    (ros::ros-info "target-x: ~A, target-y: ~A, target-theta: ~A" target-x target-y target-theta)
    (send *ri* :go-pos-no-wait target-x target-y (deg2rad target-theta))
    )
  (if *use-ros-lock*
      (send *ros-lock* :release)
      )
  )

(exit)
