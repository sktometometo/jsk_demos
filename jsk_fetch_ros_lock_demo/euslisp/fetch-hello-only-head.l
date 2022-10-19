#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(require "package://fetcheus/fetch-interface.l")
(require "package://ros_lock/euslisp/ros_lock.l")
(require "package://jsk_fetch_ros_lock_demo/euslisp/utils.l")

(ros::roseus "fetch_hello_only_head")
(jsk-fetch-ros-lock-init)

(setq *threshold-distance* (ros::get-param "~threshold_distance" 5.0))
(setq *threshold-delay* (ros::get-param "~threshold_delay" 2.0))
(setq *duration-dead* (ros::get-param "~duration_dead" 2.0))

(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (ros::sleep)
  (ros::ros-info "*target-person-coord-robotbased-data*: ~A" *target-person-coord-robotbased-data*)
  (ros::ros-info "*target-person-coord-robotbased-stamp*: ~A" *target-person-coord-robotbased-stamp*)
  (if *target-person-coord-robotbased-data*
    (let* (ret
           (target-y (* 0.001 (elt (send *target-person-coord-robotbased-data* :pos) 1)))
           (target-distance (* 0.001 (norm (send *target-person-coord-robotbased-data* :pos))))
           (delay (ros::time- (ros::time-now) *target-person-coord-robotbased-stamp*))
           (duration-from-last (ros::time- (ros::time-now) *last-stamp-approach*))
           )
      (ros::ros-info "target-y: ~A" target-y)
      (ros::ros-info "target-distance: ~A" target-distance)
      (ros::ros-info "delay: ~A" delay)
      (ros::ros-info "duration-from-last: ~A" duration-from-last)
      ;; Acquire lock
      (setq ret (send *ros-lock* :acquire "head" :timeout 10))
      (ros::spin-once)
      (if (and (< target-distance *threshold-distance*)
               (ros::time< delay (send (instance ros::time :init) :from-sec *threshold-delay*))
               (ros::time> duration-from-last (send (instance ros::time :init) :from-sec *duration-dead*))
               (> target-y 0.2)
               ret
               )
        (progn
          ;; Say hello
          (ros::ros-info "*target-person-coord-robotbased-data*: ~A" *target-person-coord-robotbased-data*)
          ;;(send *ri* :speak-jp "こんにちは")
          (let ((deadline (ros::time+ (ros::time-now) (send (instance ros::time :init) :from-sec 5))))
            (while (and (ros::ok)
                        *target-person-coord-robotbased-data*
                        (ros::time< (ros::time-now) deadline)
                        )
              (ros::ros-info "point head to ~A" *target-person-coord-robotbased-data*)
              (send *ri* :point-head (send *target-person-coord-robotbased-data* :pos) :wait t)
              (ros::spin-once)
              )
            )
          (send *ri* :point-head #f(600 0 1000) :wait t)
          ;;
          (setq *last-stamp-approach* (ros::time-now))
          )
        )
      ;; Release lock
      (send *ros-lock* :release "head")
      (ros::ros-error "released")
      )
    (ros::ros-error "Unable to find person")
    ))
