#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch-approach-person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))

(setq *ready-pose* #f(47.7909 75.6931 80.1877 -11.5039 98.5518 -24.6569 108.319 0.015018 0.447521 0.501636))
(setq *hold-out-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))
(setq *pre-get-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))
(setq *get-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))
(setq *pro-get-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))
(setq *pre-put-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))
(setq *put-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))
(setq *pro-put-pose* #f(47.8061 56.7087 58.8303 -57.6904 107.385 -45.3991 90.9389 -0.006954 0.300235 0.523609))

(setq *ros-lock-for-head* (instance ros-lock :init "head"))
(send *ros-lock-for-head* :wait-for-server)
(setq *ros-lock-for-arm* (instance ros-lock :init "arm"))
(send *ros-lock-for-arm* :wait-for-server)
(setq *ros-lock-for-navigation* (instance ros-lock :init "navigation"))
(send *ros-lock-for-navigation* :wait-for-server)

(setq *nearest-people-coord-data* nil)
(setq *nearest-people-coord-stamp* nil)

(defun get-basket ()
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *fetch* :angle-vector *pre-get-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *fetch* :angle-vector *get-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :start-grasp)
  (send *fetch* :angle-vector *pro-get-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *fetch* :angle-vector *ready-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  t)

(defun hold-out-basket ()
  (send *fetch* :angle-vector *hold-out-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  t)

(defun ready-basket ()
  (send *fetch* :angle-vector *ready-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  t)

(defun put-basket ()
  (send *fetch* :angle-vector *ready-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *fetch* :angle-vector *pre-put-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *fetch* :angle-vector *put-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :stop-grasp)
  (send *fetch* :angle-vector *pro-put-pose*)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  t)

(defun callback (msg)
  (let* (bbox-coord
         (bbox-frame-id (send msg :header :frame_id))
         (coord-base-to-bbox
            (send *tfl* :lookup-transform
                  *base-frame-id*
                  bbox-frame-id
                  (ros::time)))
         (people-coords-fixed-based nil)
         )
    (if (not (send msg :boxes))
      (progn
        (setq *nearest-people-coord-data* nil)
        (setq *nearest-people-coord-stamp* (send msg :header :stamp))
        )
      (if coord-base-to-bbox
        (progn
          (setq people-coords-base-based
                  (mapcar
                     #'(lambda (bbox)
                         (transform-coords
                           coord-base-to-bbox
                           (ros::tf-pose->coords (send bbox :pose))))
                     (send msg :boxes))
                )
          (setq *nearest-people-coord-data*
              (elt
                (sort people-coords-base-based
                      #'(lambda (coord-a coord-b)
                          (<= (norm (send coord-a :pos))
                              (norm (send coord-b :pos))
                              )
                          ))
                0))
          (setq *nearest-people-coord-stamp* (send msg :header :stamp))
          ))
      )
    ))

(ros::subscribe "~input_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback)

(while (ros:ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (if (and  (ros::time<
              (ros::time- (ros::time-now) *nearest-people-coord-stamp*)
              (send (instance ros::time :init) :from-sec 5.0)
              )
            (<= (norm (send *nearest-people-coord-data* :pos)) *max-threshold-distance*)
            (>= (norm (send *nearest-people-coord-data* :pos)) *min-threshold-distance*)
            )
    (let* ((pos-x (elt (send *nearest-people-coord-data* :pos) 0))
           (pos-y (elt (send *nearest-people-coord-data* :pos) 1))
           (distance (norm (send *nearest-people-coord-data* :pos)))
           (target-x (* pos-x (/ (- distance 0.5) distance)))
           (target-y (* pos-y (/ (- distance 0.5) distance)))
           (target-theta (atan pos-y pos-x))
           )
      ;; Acquire lock
      (send *ros-lock-for-head* :acquire)
      (send *ros-lock-for-arm* :acquire)
      (send *ros-lock-for-navigation* :acquire)
      ;; Hold out basket to person
      (send *ri* :go-pos-no-wait 0 0 (rad2deg target-theta))
      (get-basket)
      (hold-out-basket)
      (send *ri* :speak-jp "お菓子はいかがですか")
      (unix:sleep 5)
      (ready-basket)
      (put-basket)
      ;; Release lock
      (send *ros-lock-for-head* :release)
      (send *ros-lock-for-arm* :release)
      (send *ros-lock-for-navigation* :release)
      )
    ))
