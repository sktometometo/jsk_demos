#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::load-ros-package "uwb_localization")


(setq *odom-vision-to-body* nil)
(setq *last-msg-table* (make-hash-table :test 'equal))

(setq *threshold-distance-max* 5.0)
(setq *timeout-duration-move* 3.0)
(setq *duration-dead-zone* 60)

(setq *ros-lock-mobility* (instance ros-lock :init "mobility"))
(setq *ros-lock-arm* (instance ros-lock :init "arm"))


(defun callback-odom (msg)
  (setq *odom-vision-to-body*
    (ros::tf-pose->coords
      (send msg :pose :pose))))

(defun callback-devices (msg)
  (if (or (not *odom-vision-to-body*)
          (= 0 (length (send msg :devices))))
    (return-from callback-devices))
  (let* (
         (target-device
           (car
             (sort
               (send msg :devices)
               #'(lambda (device-a device-b)
                 (<
                   (distance
                     (ros::tf-point->pos (send device-a :point))
                     (send *odom-vision-to-body* :pos)
                     )
                   (distance
                     (ros::tf-point->pos (send device-b :point))
                     (send *odom-vision-to-body* :pos)
                     ))))))
         (target-device-coords-from-robot
           (transform-coords
             (send *odom-vision-to-body* :inverse-transformation)
             (make-coords :pos (ros::tf-point->pos (send target-device :point))))
           )
         (target-device-name (send target-device :device_name))
         (last-stamp-for-device (gethash target-device-name *last-msg-table*))
         (distance-robot-to-device
           (norm (send target-device-coords-from-robot :pos)))
         )

    (if (> distance-robot-to-device *threshold-distance-max*)
      (progn
        (ros::ros-error (format nil "Distance: ~A is larger then threshold: ~A"
                                distance-robot-to-device
                                *threshold-distance-max*
                                ))
        (return-from callback-devices))
      )

    (if (or (not last-stamp-for-device)
            (ros::time>
              (ros::time- (ros::time-now) last-stamp-for-device)
              (ros::time *duration-dead-zone*)))
      (progn
        (point-and-describe
          target-coords-from-robot
          :name (send target-device :device_name)
          :description ""
          )
        (setf (gethash device-name *last-msg-table*) (send target-device :header :stamp))
        )
      )
    ))

(defun point-and-describe
    (target-coords-from-robot
     &key
       (name "test-name")
       (description "test-description")
       )
  (let* ((local-x (elt (send target-coords-from-robot :pos) 0))
         (local-y (elt (send target-coords-from-robot :pos) 1))
         (local-z (elt (send target-coords-from-robot :pos) 2))
         (direction (rad2deg (atan local-y local-x)))
         (distance2point (norm (float-vector local-x local-y 0)))
         )
    (ros::ros-info (format nil "direction: ~A" direction))
    (ros::ros-info (format nil "distance: ~A" distance2point))
    (ros::ros-info (format nil "local-x: ~A" local-x))
    (ros::ros-info (format nil "local-y: ~A" local-y))
    (ros::ros-info (format nil "local-z: ~A" local-z))
    (send *ros-lock-mobility* :acquire)
    (send *ros-lock-arm* :acquire)
    (send *ri* :go-pos 0 0 direction :wait t)
    (send *spot* :angle-vector (send *ri* :state :potentio-vector))
    (send *spot* :reset-pose)
    (send *spot* :head :look-at (float-vector distance2point 0 local-z))
    (send *ri* :angle-vector (send *spot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *ros-lock-mobility* :release)
    (send *ros-lock-arm* :release)
    (send *ri* :speak (format nil "There is ~A there." name) :wait t)
    (send *ri* :speak description :wait t)
    )
  )

(defun main ()
  (spot-init)
  (send *ros-lock-mobility* :release)
  (send *ros-lock-arm* :release)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/sdpuwb_devices" uwb_localization::SDPUWBDeviceArray #'callback-devices)
  (ros::spin)
  )


(main)
