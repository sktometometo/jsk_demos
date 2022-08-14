#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch-look-at-person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *robot-frame-id* (ros::get-param "~robot_frame_id" "base_link"))

(setq *ros-lock-for-head* (instance ros-lock :init "head"))
(send *ros-lock-for-head* :wait-for-server)
(setq *ros-lock-for-navigation* (instance ros-lock :init "navigation"))
(send *ros-lock-for-navigation* :wait-for-server)

(setq *people-coords-robot-based* nil)

(defun callback (msg)
  (let* (bbox-coord
         (frame-id (send msg :header :frame_id))
         (coords-robot-to-fixed
            (send *tfl* :lookup-transform
                  *base-frame-id*
                  frame-id
                  (ros::time)))
         )
    (setq *people-coords-robot-based* (mapcar
                   #'(lambda (bbox)
                       (transform-coords
                         coords-robot-to-fixed
                         (ros::tf-pose->coords (send bbox :pose))))
                   (send msg :boxes)))
    ))

(ros::subscribe "~input_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback)

(while (ros:ok)
  (ros::spin-once)
  (send *ros-lock-for-head* :acquire)
  (send *ros-lock-for-navigation* :acquire)
  (while (ros::ok)
      (send *go-pos-no-wait*
        (- (random (- *max-x-range* *min-x-range*)) *min-x-range*)
        (- (random (- *max-y-range* *min-y-range*)) *min-y-range*)
        (rad2deg (- (random (- *max-theta-range* *min-theta-range*)) *min-theta-range*))
        )
      )
  (send *ros-lock-for-head* :release)
  (send *ros-lock-for-navigation* :release)
  )
