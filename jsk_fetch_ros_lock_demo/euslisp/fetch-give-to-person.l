#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch-give-to-person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))
(setq *threshold-distance* (ros::get-param "~threshold_distance" 5.0))

(setq *ros-lock-for-head* (instance ros-lock :init "head"))
(send *ros-lock-for-head* :wait-for-server)
(setq *ros-lock-for-navigation* (instance ros-lock :init "navigation"))
(send *ros-lock-for-navigation* :wait-for-server)

(setq *nearest-people-coord-data* nil)
(setq *nearest-people-coord-stamp* nil)

(defun callback (msg)
  (let* (bbox-coord
         (bbox-frame-id (send msg :header :frame_id))
         (coord-fixed-to-bbox-frame
            (send *tfl* :lookup-transform
                  *fixed-frame-id*
                  bbox-frame-id
                  (ros::time)))
         (coord-fixed-to-base
            (send *tfl* :lookup-transform
                  *fixed-frame-id*
                  *base-frame-id*
                  (ros::time)))
         (people-coords-fixed-based nil)
         )
    (if (and coord-fixed-to-bbox-frame coord-fixed-to-base)
      (progn
        (setq people-coords-fixed-based
                (mapcar
                   #'(lambda (bbox)
                       (transform-coords
                         coords-robot-to-fixed
                         (ros::tf-pose->coords (send bbox :pose))))
                   (send msg :boxes))
              )
        (setq *nearest-people-coord-data*
              (sort people-coords-fixed-based
                    #'(lambda (coord-a coord-b)
                        (<= (norm (v- (send coord-a :pos) (send coord-fixed-to-base :pos)))
                            (norm (v- (send coord-b :pos) (send coord-fixed-to-base :pos))))
                        )))
        (setq *nearest-people-coord-stamp* (send msg :header :stamp))
        ))
    ))

(ros::subscribe "~input_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback)

(ros::rate 1)
(while (ros:ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (ros::sleep)
  (if (and  (ros::time>
              (ros::time- (ros::time-now) *nearest-people-coord-stamp*)
              (ros::time-from))
            ()
            )
    (progn
      (send *ros-lock-for-head* :acquire)
      (send *ros-lock-for-navigation* :acquire)
      (while (ros::ok)
        (let* ()
          (ros::spin-once)
          (send *ri* :spin-once)
          ))
      (send *ros-lock-for-head* :release)
      (send *ros-lock-for-navigation* :release)
      )
    ))
