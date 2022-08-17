#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch_approach_person")
(setq *ri* (instance fetch-interface :init :move-base-action-name "move_base_no_recovery"))
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))
(setq *min-threshold-distance* (ros::get-param "~min_threshold_distance" 1.5))
(setq *max-threshold-distance* (ros::get-param "~max_threshold_distance" 5.0))

(setq *ros-lock-for-head* (instance ros-lock :init "head"))
(send *ros-lock-for-head* :wait-for-server)
(setq *ros-lock-for-navigation* (instance ros-lock :init "navigation"))
(send *ros-lock-for-navigation* :wait-for-server)

(setq *nearest-people-coord-data* nil)
(setq *nearest-people-coord-stamp* nil)

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
      (send *ros-lock-for-navigation* :acquire)
      ;; aproach to person
      (send *ri* :go-pos-no-wait target-x target-y (rad2deg target-theta))
      (send *ri* :point-head (send *nearest-people-coord-data* :pos) :wait nil)
      (unix::sleep 1)
      ;; Release lock
      (send *ros-lock-for-head* :release)
      (send *ros-lock-for-navigation* :release)
      )
    ))
