#!/usr/bin/env python

(load "package://spoteus/spot-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "uwb_localization")

(setq *nearest-person-coords* nil)
(setq *nearest-device-coords* nil)
(setq *odom-vision-to-body* nil)

(setq *ros-lock-mobility* nil)

(defun callback-odom (msg)
  (setq *odom-vision-to-body*
    (ros::tf-pose->coords
      (send msg :pose :pose))))

(defun callback-people (msg-bbox-array)
  (let ()
    )
  )

(defun callback-device (msg-array)
  (let (device-coords-from-robot
        nearest-coords-from-robot)
    (setq device-coords-from-robot
          (mapcar
           #'(lambda (msg)
               (send msg-array :point))
           (send msg-array :devices)))
    (setq
     nearest-coords-from-robot
     (first
      (sort
       device-coords-from-robot
       #'(lambda (x y) (<
                        (norm (send c :pos))
                        (norm (send c :pos)))))))
    (if (and nearest-coords-from-robot *odom-vision-to-robot*)
      (setq *nearest-device-coords*
            (transform-coords *odom-vision-to-robot* nearest-coords-from-robot))
      (setq *nearest-device-coords* nil))
  ))

(defun main ()
  (spot-init)

  (setq *ros-lock* (instance ros-lock :init "mobility"))

  (ros::subscribe
   "/spot_recognition/bbox_aray"
   jsk_recognition_msgs::BoundingBoxArray
   #'callback-people)
  (ros::subscribe
   "/sdpuwb_devices"
   uwb_localization::SDPUWBDeviceArray
   #'callback-device)
  (ros::subscribe
   "/spot/odometry"
   nav_msgs::Odometry
   #'callback-odom)

  (ros::rate 1)
  (while (ros::ok)
    (ros::sleep)
    (ros::spin-once)
    (let (target)
      (if (and *odom-vision-to-robot* *nearest-device-coords*)
          (progn
            (send *ros-lock* :acquire)
            (setq target
                  (transform-coords
                   (send *odom-vision-to-robot* :inverse-transformation)
                   *nearest-device-coords*))
            (send *ri* :untuck-arm)
            (send *spot* :angle-vector (send *ri* :state :potentio-vector))
            (send *spot* :arm :look-at (send target :pos))
            (send *ros-lock* :release)
            )
          )
      )
    )
  )

