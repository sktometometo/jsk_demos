#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch_approach_person")
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
    (let* (ret-head ret-arm ret-ik
           (pos-x (* 0.001 (elt (send *target-person-coord-robotbased-data* :pos) 0)))
           (pos-y (* 0.001 (elt (send *target-person-coord-robotbased-data* :pos) 1)))
           (distance (* 0.001 (norm (send *target-person-coord-robotbased-robotbased-data* :pos))))
           (target-x (* pos-x (/ (- distance 0.5) distance)))
           (target-y (* pos-y (/ (- distance 0.5) distance)))
           (target-theta (atan pos-y pos-x))
           (delay (ros::time- (ros::time-now) *target-person-coord-robotbased-stamp*))
           (duration-from-last (ros::time- (ros::time-now) *last-stamp-approach*))
           )
      (ros::ros-info "distance: ~A" distance)
      (ros::ros-info "delay: ~A" delay)
      (if (and (< distance *threshold-distance*)
               (ros::time< delay (send (instance ros::time :init) :from-sec *threshold-delay*))
               (ros::time> duration-from-last (send (instance ros::time :init) :from-sec *duration-dead*))
               )
        (progn
          ;; Acquire lock
          (setq ret-head (send *ros-lock* :acquire "head" :timeout 10))
          (setq ret-arm (send *ros-lock* :acquire "arm" :timeout 10))
          (if (and (not ret-head) (not ret-arm))
            (progn
              (ros::ros-error "Unable to acquire lock")
              (exit 1)
              )
            )
          ;; hello with hand
          (ros::ros-info "*target-person-coord-robotbased-data*: ~A" *target-person-coord-robotbased-data*)
          ;;; move hand once
          (send *ri* :spin-once)
          (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
          (setq ret-ik
            (send *fetch* :inverse-kinematics
                (transform-coords
                  (send (send *fetch* :head :end-coords) :copy-worldcoords)
                  (make-coords :pos #f(16.951 -121.855 172.07) :rpy #f(1.941 -0.361 -1.249)))
                :use-torso nil)
            )
          (if ret-ik
            (progn
              (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
              (send *ri* :wait-interpolation)
              ))
          ;;; say hello and move hand
          (send *ri* :speak-jp "こんにちは")
          (let ((deadline (ros::time+ (ros::time-now) (send (instance ros::time :init) :from-sec 10))))
            (while (and (ros::ok)
                        *target-person-coord-robotbased-data*
                        (ros::time< (ros::time-now) deadline)
                        )
              (send *ri* :point-head (send *target-person-coord-robotbased-data* :pos) :wait t)
              (send *ri* :spin-once)
              (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
              (setq ret-ik
                (send *fetch* :inverse-kinematics
                    (transform-coords
                      (send (send *fetch* :head :end-coords) :copy-worldcoords)
                      (make-coords :pos #f(16.951 -121.855 172.07) :rpy #f(1.941 -0.361 -1.249)))
                    :use-torso nil)
                )
              (if ret-ik
                (progn
                  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
                  (send *ri* :wait-interpolation)
                  ))
              (ros::spin-once)
              )
            )
          ;; Release lock
          (send *ros-lock* :release "head")
          (send *ros-lock* :release "arm")
          (ros::ros-error "released")
          ;;
          (setq *last-stamp-approach* *target-person-coord-robotbased-stamp*)
          )
        )
      )
    (ros::ros-error "Unable to find person")
    ))
