#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(require "package://fetcheus/fetch-interface.l")
(require "package://ros_lock/euslisp/ros_lock.l")
(require "package://jsk_fetch_ros_lock_demo/euslisp/utils.l")

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
           (target-distance (* 0.001 (norm (send *target-person-coord-robotbased-robotbased-data* :pos))))
           (delay (ros::time- (ros::time-now) *target-person-coord-robotbased-stamp*))
           (duration-from-last (ros::time- (ros::time-now) *last-stamp-approach*))
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
          (if (or (not ret-head) (not ret-arm))
            (progn
              (ros::ros-error "Unable to acquire lock")
              (exit 1)
              )
            )
          ;; hello with hand
          (ros::ros-info "*target-person-coord-robotbased-data*: ~A" *target-person-coord-robotbased-data*)
          ;;; move hand once
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
          ;;; say hello and move hand
          (send *ri* :speak-jp "こんにちは")
          (let ((count 0)
                (deadline (ros::time+ (ros::time-now) (send (instance ros::time :init) :from-sec 10))))
            (while (and (ros::ok)
                        *target-person-coord-robotbased-data*
                        (ros::time< (ros::time-now) deadline)
                        )
              (send *ri* :point-head (send *target-person-coord-robotbased-data* :pos) :wait nil)
              (send *ri* :spin-once)
              (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
              (if (oddp count)
                  (progn
                    (setq ret-ik
                          (send *fetch* :inverse-kinematics
                                (transform-coords
                                 (send (send *fetch* :head :end-coords) :copy-worldcoords)
                                 (make-coords :pos #f(-16.755 -130.538 142.025) :rpy #f(2.035 -0.205 -0.15)))
                                :use-torso nil))
                    )
                  (progn
                    (setq ret-ik
                          (send *fetch* :inverse-kinematics
                                (transform-coords
                                 (send (send *fetch* :head :end-coords) :copy-worldcoords)
                                 (make-coords :pos #f(-134.649 -293.126 99.766) :rpy #f(2.552 0.004 -0.03)))
                                :use-torso nil))
                    )
                  )
              (setq count (+ 1 count))
              (if ret-ik
                (progn
                  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
                  (send *ri* :wait-interpolation)
                  ))
              (ros::spin-once)
              )
            )
          ;; Release lock
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
