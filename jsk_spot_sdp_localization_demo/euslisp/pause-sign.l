#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::load-ros-package "uwb_localization")


(setq *odom-vision-to-body* nil)


(setq *ros-lock* (instance ros-lock :init "mobility"))


(defun callback-odom (msg)
  (setq *odom-vision-to-body*
    (ros::tf-pose->coords
      (send msg :pose :pose))))

(defun convert-point-to-robot-based (ros-point)
  (transform-coords
    (send *odom-vision-to-body* :inverse-transformation)
    (make-coords :pos (ros::tf-point->pos (send target-device :point))))
  )

(defun callback (msg)
  (if (or (not *odom-vision-to-body*)
          (= 0 (length (send msg :devices))))
    (return-from callback))
  (let* ((sorted-list
           (send msg :devices)
           (lambda (device-a device-b)
             (<
               (distance
                 (ros::tf-point->pos (send device-a :point))
                 (send *odom-vision-to-body* :pos)
                 )
               (distance
                 (ros::tf-point->pos (send device-b :point))
                 (send *odom-vision-to-body* :pos)
                 ))))
         (target-device (elt sorted-list 0))
         (target-coords-from-robot
           (transform-coords
             (send *odom-vision-to-body* :inverse-transformation)
             (make-coords :pos (ros::tf-point->pos (send target-device :point))))
           )
         (device-name (send target-device :device_name))
         (last-stamp-for-device (gethash device-name *last-msg-table*))
         (distance-robot-to-device
           (norm (send target-coords-from-robot :pos)))
         )
    ))


(defun main ()
  (spot-init)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/sdpuwb_devices" uwb_localization::SDPUWBDeviceArray #'callback)
  (while (ros::ok)
    (ros::spin-once)
    )
  )
