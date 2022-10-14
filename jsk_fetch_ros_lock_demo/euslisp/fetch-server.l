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

(setq *pre-grasp-pose-torso-based* (make-coords :pos #f(120.07 -205.033 148.356) :rpy #f(-3.069 0.373 0.029)))
(setq *grasp-pose-torso-based* (make-coords :pos #f(53.8 -238.317 109.98) :rpy #f(-3.085 0.412 -0.018)))
(setq *pro-grasp-pose-01-torso-based* (make-coords :pos #f(72.42 -248.861 185.423) :rpy #f(-3.024 0.197 0.403)))
(setq *pro-grasp-pose-02-torso-based* (make-coords :pos #f(63.43 -370.283 184.052) :rpy #f(-2.633 0.152 0.488)))
(setq *ready-pose* (make-coords :pos #f(363.334 -139.026 500) :rpy #f(0 0 0)))

(defun get-current-end-coords-torso-based ()
  (let (torso-link-coords end-coords-base-based)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
    (setq torso-link-coords (send (send *fetch* :link "torso_lift_link") :copy-worldcoords))
    (setq end-coords-base-based (send (send *fetch* :rarm :end-coords) :copy-worldcoords))
    (transform-coords (send torso-link-coords :inverse-transformation) end-coords-base-based)
    )
  )

(defun transform-pose-to-base-based (coords)
  (let (torso-link-coords)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
    (setq torso-link-coords (send (send *fetch* :link "torso_lift_link") :copy-worldcoords))
    (transform-coords torso-link-coords coords)
    )
  )

(defun get-basket ()
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *pre-grasp-pose-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp)
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *grasp-pose-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *pro-grasp-pose-01-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *pro-grasp-pose-02-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *fetch* :inverse-kinematics *ready-pose* :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun put-basket ()
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *pro-grasp-pose-02-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *pro-grasp-pose-01-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *grasp-pose-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp)
  (send *fetch* :inverse-kinematics (transform-pose-to-base-based *pre-grasp-pose-torso-based*) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

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
          ;;
          (send *ri* :go-pos destination-x destination-y (rad2deg target-direction))
          (send *ri* :spin-once)
          ;; Hold out basket to person
          (get-basket)
          ;;
          ;(setq path-constraints
          ;      (instance moveit_msgs::constraints :init
          ;                :name ""
          ;                :orientation_constraints
          ;                (list (instance moveit_msgs::OrientationConstraint :init
          ;                                :header
          ;                                (instance std_msgs::Header :init :frame_id "base_link")
          ;                                :link_name "gripper_link"
          ;                                :orientation
          ;                                (instance geometry_msgs::Quaternion :init
          ;                                          :x 0 :y 0 :z 0 :w 1.0)
          ;                                :absolute_x_axis_tolerance 2.0
          ;                                :absolute_y_axis_tolerance 2.0
          ;                                :absolute_z_axis_tolerance 3.15))
          ;                ))
          ;(send *fetch* :inverse-kinematics *ready-pose* :use-torso t)
          ;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :default-controller 0 :path-constraints path-constraints)
          ;;
          (send *ri* :speak-jp "お菓子はいかがですか")
          ;;
          (put-basket)
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
