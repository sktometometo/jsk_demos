#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(ros::load-ros-package "uwb_localization")


(setq *odom-vision-to-body* nil)
(setq *last-moved-stamp* nil)
(setq *last-msg-table* (make-hash-table :test 'equal))


(defun callback-odom (msg)
  (setq *odom-vision-to-body*
    (ros::tf-pose->coords
      (send msg :pose :pose))))

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
         (device-name (send target-device :device_name))
         (last-stamp-for-devide (gethash device-name *last-msg-table*))
         (distance-robot-to-device
           (norm
             (send
               (transform-coords
                    (send *odom-vision-to-body* :inverse-transformation)
                    (make-coords :pos (ros::tf-point->pos (send target-device :point))))
               :pos)))
         )
    (if (or (not last-stamp-for-devide)
            (ros::time> (ros::time- (ros::time-now) last-stamp-for-devide) (ros::time 30)))
      (progn
        (send *ri* :speak-jp (format nil "あそこにあるのは~Aです" device-name))
        (setf (gethash device-name *last-msg-table*) (send target-device :header :stamp))
        ))

    (ros::ros-info (format nil "Target device is ~A" (send target-device :device_name)))
    (point-and-describe
      (transform-coords
        (send *odom-vision-to-body* :inverse-transformation)
        (make-coords :pos (ros::tf-point->pos (send target-device :point))))
      (send target-device :device_name)
      ""
      :walk nil
      :speak nil)
    (setq *last-moved-stamp* (send target-device :header :stamp))
   ))

(defun point-and-describe
    (target-coords-from-robot name description
     &key
       (walk t)
       (speak t)
       )
  (let* ((local-x (elt (send target-coords-from-robot :pos) 0))
         (local-y (elt (send target-coords-from-robot :pos) 1))
         (local-z (elt (send target-coords-from-robot :pos) 1))
         (direction (rad2deg (atan local-y local-x)))
         (distance2point (norm (float-vector local-x local-y 0)))
         )
    (ros::ros-info (format nil "direction: ~A" direction))
    (ros::ros-info (format nil "distance: ~A" distance2point))
    (if walk
        (progn
          (send *ri* :go-pos 0 0 direction :wait t)
          (send *spot* :angle-vector (send *ri* :state :potentio-vector))
          (send *spot* :head :look-at (float-vector distance2point 0 local-z))
          (send *ri* :angle-vector (send *spot* :angle-vector) 5000))
        (progn
          (send *spot* :angle-vector (send *ri* :state :potentio-vector))
          (send *spot* :head :look-at (float-vector local-x local-y local-z))
          (send *ri* :angle-vector (send *spot* :angle-vector) 1000))
          )
    (if speak
      (progn
        (send *ri* :speak-jp (format nil "あそこにあるのは~Aです" name) :wait t)
        (send *ri* :speak-jp description :wait t)))
    ))

(defun main ()
  (spot-init)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/sdpuwb_devices" uwb_localization::SDPUWBDeviceArray #'callback)
  (while (ros::ok)
    (ros::spin-once)
    )
  )
