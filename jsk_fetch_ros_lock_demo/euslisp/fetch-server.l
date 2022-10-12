#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(require "package://fetcheus/fetch-interface.l")
(require "package://ros_lock/euslisp/ros_lock.l")
(require "package://jsk_fetch_ros_lock_demo/euslisp/utils.l")

(ros::roseus "fetch_server")
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
  (if (and *target-person-coord-robotbased-data* *target-person-coord-robotbased-stamp*)
    (let* (ret-head ret-arm ret-base
           (target-x (* 0.001 (elt (send *target-person-coord-robotbased-data* :pos) 0)))
           (target-y (* 0.001 (elt (send *target-person-coord-robotbased-data* :pos) 1)))
           (target-direction (atan target-y target-x))
           (target-distance (* 0.001 (norm (send *target-person-coord-robotbased-data* :pos))))
           (delay (ros::time- (ros::time-now) *target-person-coord-robotbased-stamp*))
           (duration-from-last (ros::time- (ros::time-now) *last-stamp-approach*))
           (destination-x (* target-x (/ (- target-distance 0.5) target-distance)))
           (destination-y (* target-y (/ (- target-distance 0.5) target-distance)))
           )
      (ros::ros-info "target-distance: ~A" target-distance)
      (ros::ros-info "delay: ~A" delay)
      (ros::ros-info "duration-from-last: ~A" duration-from-last)
      (if (and (< target-distance *threshold-distance*)
               (ros::time< delay (send (instance ros::time :init) :from-sec *threshold-delay*))
               (ros::time> duration-from-last (send (instance ros::time :init) :from-sec *duration-dead*))
               )
        (progn
          ;; Acquire lock
          (setq ret-head (send *ros-lock* :acquire "head" :timeout 10))
          (setq ret-arm (send *ros-lock* :acquire "arm" :timeout 10))
          (setq ret-base (send *ros-lock* :acquire "base" :timeout 10))
          (if (or (not ret-head) (not ret-arm) (not ret-base))
            (progn
              (ros::ros-error "Unable to acquire lock for base")
              (exit 1)
              ))
          ;; Hold out basket to person
          (send *ri* :go-pos destination-x destination-y (rad2deg target-direction))
          (send *ri* :speak-jp "お菓子はいかがですか")
          ;; Release lock
          (send *ros-lock* :release "base")
          (send *ros-lock* :release "arm")
          (send *ros-lock* :release "head")
          (ros::ros-error "released")
          ;;
          (setq *last-stamp-approach* *target-person-coord-robotbased-stamp*)
          )
        )
      )
      (ros::ros-error "Unable to find person")
    ))
