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
(setq *min-threshold-distance* (ros::get-param "~min_threshold_distance" 0.5))
(setq *max-threshold-distance* (ros::get-param "~max_threshold_distance" 3.0))

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

(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (ros::sleep)
  (ros::ros-info "*nearest-people-coord-data*: ~A" *nearest-people-coord-data*)
  (ros::ros-info "*nearest-people-coord-stamp*: ~A" *nearest-people-coord-stamp*)
  (if (and *nearest-people-coord-data* *nearest-people-coord-stamp*)
    (let* (ret
           (pos-x (elt (send *nearest-people-coord-data* :pos) 0))
           (pos-y (elt (send *nearest-people-coord-data* :pos) 1))
           (distance (* 0.001 (norm (send *nearest-people-coord-data* :pos))))
           (delay (ros::time- (ros::time-now) *nearest-people-coord-stamp*))
           (target-x (* pos-x (/ (- distance 0.5) distance)))
           (target-y (* pos-y (/ (- distance 0.5) distance)))
           (target-theta (atan pos-y pos-x))
           )
      (ros::ros-info "distance: ~A" distance)
      (ros::ros-info "delay: ~A" delay)
      (if (and (ros::time<
                 delay
                 (send (instance ros::time :init) :from-sec 5.0)
                 )
               (<= distance *max-threshold-distance*)
               (>= distance *min-threshold-distance*)
               (> pos-y 0)
               )
        (progn
          (ros::ros-error "Found person, approach to ~A ~A" target-x target-y)
          ;; Acquire lock
          (setq ret (send *ros-lock-for-head* :acquire :timeout 10))
          (if (not ret)
            (progn
              (ros::ros-error "Unable to acquire lock for head")
              (exit 1)
              )
            )
          ;;(send *ros-lock-for-navigation* :acquire)
          ;; aproach to person
          ;;(send *ri* :go-pos-no-wait target-x target-y (rad2deg target-theta))
          (while (and (ros::ok)
                      *nearest-people-coord-data*
                      (> (elt (send *nearest-people-coord-data* :pos) 1) 0)
                      )
              (ros::ros-info "*nearest-people-coord-data*: ~A" *nearest-people-coord-data*)
              (send *ri* :point-head (send *nearest-people-coord-data* :pos) :wait t)
              (unix::sleep 1)
              (ros::spin-once)
              )
          (send *ri* :point-head #f(1.0 0 1.0) :wait nil)
          ;; Release lock
          ;;(send *ros-lock-for-navigation* :release)
          (send *ros-lock-for-head* :release)
          (ros::ros-error "released")
          )
        )
      )
    (ros::ros-error "Person not found.)
    ))
