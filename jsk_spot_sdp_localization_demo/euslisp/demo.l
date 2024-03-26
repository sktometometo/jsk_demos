#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(ros::load-ros-package "uwb_localization")

(defun callback (msg)
  (let ()
   ))

(defun point-and-describe
    (target-coords-from-robot name description
     &key
       (walk t)
       )
  (let* ((local-x (elt (send target-coords-from-robot :pos) 0))
         (local-y (elt (send target-coords-from-robot :pos) 1))
         (local-z (elt (send target-coords-from-robot :pos) 1))
         (direction (rad2deg (atan local-y local-x)))
         (distance (norm (float-vector local-x local-y 0)))
         )
    (ros::ros-info (format nil "direction: ~A" direction))
    (ros::ros-info (format nil "distance: ~A" distance))
    (if walk
        (progn
          (send *ri* :go-pos 0 0 direction :wait t)
          (send *spot* :angle-vector (send *ri* :state :potentio-vector))
          (send *spot* :head :look-at (float-vector distance 0 local-z))
          (send *ri* :angle-vector (send *spot* :angle-vector) 5000))
        (progn
          (send *spot* :angle-vector (send *ri* :state :potentio-vector))
          (send *spot* :head :look-at (float-vector local-x local-y local-z))
          (send *ri* :angle-vector (send *spot* :angle-vector) 1000))
          )
    (send *ri* :speak-jp (format nil "あそこにあるのは~Aです" name) :wait t)
    (send *ri* :speak-jp description :wait t)
    ))

(defun main ()
  (spot-init)

  (ros::subscribe "/sdpuwb_devices" uwb_localization::SDPUWBDeviceArray #'callback)

  (ros::spin)
  )
